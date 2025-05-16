/*
Motor:
Power: 12V
Horizontal: 1.5A, 4 Microstep
Vertical: 1.5A, 4 Microstep

V0.1.3:change to 24bit bmp
V0.2.1:add S shape, add raw file, motor smooth start
Jimmy Zhang
2025.4
*/
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <SD.h>
#include <TFT_eSPI.h>
#include <Adafruit_TCS34725.h>
#include <VTI7064.h>
#include <TimerOne.h>

//Pin definition
#define MOTOR_HORIZONTAL 0
#define MOTOR_VERTICAL 1
#define MOTOR_FORWARD 1
#define MOTOR_BACKWARD 0
#define MOTOR_HORIZONTAL_PUL 2
#define MOTOR_HORIZONTAL_DIR 3
#define MOTOR_HORIZONTAL_ENA 4
#define MOTOR_VERTICAL_PUL 5
#define MOTOR_VERTICAL_DIR 6
#define MOTOR_VERTICAL_ENA 7
//Move parameters
#define HORIZONTAL_INTERVAL_TIME 350
#define VERTICAL_INTERVAL_TIME 350
#define HORIZONTAL_PULSE_PER_PIXEL 800 //Minimum 80 for 1204 and TCS34725, 4 microstep
#define VERTICAL_PULSE_PER_PIXEL 800

//Motor data
struct Motor {
  uint16_t pulsePerPixel;
  uint16_t intervalTime;
  uint8_t millimeterPerPulse;
} motor[2];

struct {
  uint8_t pin;
  volatile uint16_t count = 0;
  uint16_t pulsePerPixel;
  uint16_t intervalTime;
} timer;

//Scan data, constantly in the scanning process
struct ScanParam {
  uint8_t path;
  uint16_t totalLines;  //For movement
  uint16_t pixelsPerLine;

  uint8_t pixelMotor;
  uint8_t pixelDirection;
  uint8_t lineMotor;
  uint8_t lineDirection;

  uint32_t filename;

  bool isSShape;

  uint8_t integrationTime = TCS34725_INTEGRATIONTIME_614MS; //Maximum interval*pulsePerPixel
  tcs34725Gain_t gain = TCS34725_GAIN_60X;
} scan;

//Comment text for path
String pathComment[8] = {
  "Right, then down",
  "Left, then down",
  "Right, then up",
  "Left, then up",
  "Down, then right",
  "Up, then right",
  "Down, then left",
  "Up, then left"
};

//For output gain information
const char* GainStr[] = {"GAIN_1X","GAIN_4X","GAIN_16X","GAIN_60X"};

//TODO:move to function
//Data for temp using, various data when scanning
struct Temp {
  uint16_t xCursor;  //Coordinate of image
  uint16_t yCursor;
  uint16_t red;  //Color data
  uint16_t green;
  uint16_t blue;
} temp;

//Other devices
File bmp;
File raw;
File name;
LiquidCrystal_I2C lcd(0x27, 16, 2);
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_60X);
TFT_eSPI tft = TFT_eSPI();
VTI7064 vti = VTI7064(9);

//Static var
int bmpHead[54] = {
  0x42, 0x4d,                                                                                     //BM word
  0x00, 0x00, 0x00, 0x00,                                                                         //File size
  0x00, 0x00, 0x00, 0x00,                                                                         //Reserve
  0x36, 0x00, 0x00, 0x00,                                                                         //File start
  0x28, 0x00, 0x00, 0x00,                                                                         //DIB header size
  0x64, 0x00, 0x00, 0x00,                                                                         //Image width(100)(18-21)
  0x64, 0x00, 0x00, 0x00,                                                                         //Image height(100)(22-25)
  0x01, 0x00,                                                                                     //Number of color plane(must be 1)
  0x18, 0x00,                                                                                     //How many bits for one pixel(24)
  0x00, 0x00, 0x00, 0x00,                                                                         //Zip method(0 for none)
  0x00, 0x00, 0x00, 0x00,                                                                         //Raw image size
  0x13, 0x0b, 0x00, 0x00, 0x13, 0x0b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00  //Other
};                                                                                                //Head data for BMP file

//temp var
uint16_t i;
uint8_t lineTemp[1000][3];//for S shape invert, order:BGR

int motorIntervalTimeShift = 0;


void setup() {
  Serial.begin(9600);
  Serial.println("Serial initialization done.");

  motorSetup();
  Serial.println("Motor initialization done.");

  lcd.begin(16, 2);
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Hello!");
  Serial.println("LCD initialization done.");

  if (!SD.begin(53)) {
    Serial.println("SD initialization failed!");
    while (1) {}
  }
  Serial.println("SD initialization done.");

  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);
  // tft.fillScreen(TFT_WHITE);
  Serial.println("TFT initialization done.");

  if (!vti.begin()) {
    Serial.println("SRAM initialization failed!");
    // while(1){}
  } else {
    Serial.println("SRAM initialization done.");
  }

  if (!tcs.begin()) {
    Serial.println("Color sensor initialization failed!");
  } else {
    Serial.println("Color sensor initialization done.");
  }

}

void loop() {

  scan.filename = getPCInt("Last filename is: " + String(scan.filename) + ". \nEnter filename:") % 100000000;//TODO:read filename from sd card
  Serial.println(scan.filename);
  scan.path = getPCInt("Enter path(0-7): ");
  Serial.println(pathComment[scan.path]);
  scan.pixelsPerLine = getPCInt("Enter pixelsPerLine:");
  Serial.println(scan.pixelsPerLine);
  scan.totalLines = getPCInt("Enter totalLines:");
  Serial.println(scan.totalLines);
  scan.isSShape = (getPCInt("S-Shape(1:Enable 0:Disable):") == 1);
  Serial.println(scan.isSShape);
  setScanMotor();
  
  //TODO:Maybe don't have to initialize SD each time, but for stable, check later
  if (!SD.begin(53)) {
    Serial.println("SD initialization failed!");
    while (1) {}
  }
  Serial.println("SD initialization done.");

  scanTask();
}

//Mainly functions
void setScanMotor() {
  //Motor select
  switch (scan.path) {
    case 0:
    case 1:
    case 2:
    case 3:
      scan.pixelMotor = MOTOR_HORIZONTAL;
      scan.lineMotor = MOTOR_VERTICAL;
      break;
    case 4:
    case 5:
    case 6:
    case 7:
      scan.pixelMotor = MOTOR_VERTICAL;
      scan.lineMotor = MOTOR_HORIZONTAL;
      break;
  }

  //Horizontal direction select
  switch (scan.path) {
    case 0:
    case 2:
    case 4:
    case 5:
      scan.pixelDirection = MOTOR_FORWARD;
      break;
    case 1:
    case 3:
    case 6:
    case 7:
      scan.pixelDirection = MOTOR_BACKWARD;
      break;
  }

  //Vertical direction select
  switch (scan.path) {
    case 0:
    case 1:
    case 4:
    case 6:
      scan.lineDirection = MOTOR_FORWARD;
      break;
    case 2:
    case 3:
    case 5:
    case 7:
      scan.lineDirection = MOTOR_BACKWARD;
      break;
  }
  
}

void scanTask() {
  
  tft.fillScreen(TFT_BLACK);

  lcd.setCursor(0, 0);
  lcd.print("Name:");
  lcd.print(scan.filename);
  lcd.print("           ");
  //Real direction
  // bmpHead[18] = scan.yPixels%256;
  // bmpHead[19] = floor(scan.yPixels/256);
  // bmpHead[22] = scan.xPixels%256;
  // bmpHead[23] = floor(scan.xPixels/256);
  //For quicker test
  bmpHead[18] = scan.pixelsPerLine % 256;
  bmpHead[19] = floor(scan.pixelsPerLine / 256);//TODO:try >>8
  bmpHead[22] = scan.totalLines % 256;
  bmpHead[23] = floor(scan.totalLines / 256);

  String dir = String(scan.filename);
  dir.concat(".bmp");
  bmp = SD.open(dir, FILE_WRITE);
  for (int i = 0; i < 54; i++) {
    bmp.write(bmpHead[i]);
  }

  dir = String(scan.filename);
  dir.concat(".raw");
  raw = SD.open(dir, FILE_WRITE);
  //Meta data, total 16 byte, 
  SDWrite16(raw, scan.pixelsPerLine); //0-1 pixelsPerLine
  SDWrite16(raw, scan.totalLines);    //2-3 totalLines
  raw.write(scan.path);               //4   path
  raw.write(scan.isSShape);           //5   S-Shape(1 for yes)
  raw.write(motor[0].pulsePerPixel);  //6   motor0 pulsePerPixel
  SDWrite16(raw, motor[0].intervalTime);//7-8
  raw.write(motor[1].pulsePerPixel);  //9
  SDWrite16(raw, motor[1].intervalTime);//10-11
  raw.write(scan.integrationTime);    //12
  raw.write(scan.gain);               //13
  raw.write(0xFF);
  raw.write(0xFF);

  //TODO:Test is it work
  tcs.setIntegrationTime(scan.integrationTime);
  Serial.println(scan.integrationTime);
  tcs.setGain(scan.gain);
  Serial.println(GainStr[scan.gain]);


  enableTimer();
  for (uint16_t i = 0; i < scan.totalLines; ++i) {
    temp.xCursor = i;

    lcd.setCursor(0, 1);
    lcd.print(i);
    lcd.print("               ");

    if(scan.isSShape && i&2 != 0) {
      setMove(scan.pixelMotor, !scan.pixelDirection, 0);
    } else {
      setMove(scan.pixelMotor, scan.pixelDirection, 0);
    }

    for (uint16_t j = 0; j < scan.pixelsPerLine; ++j) {
      temp.yCursor = j;

      //for motor smooth start
      if(j == 0 || scan.pixelsPerLine - j == 10) {
        setMoveRetard(scan.pixelMotor, 100);
      }
      if(j == 10) {
        setMoveRetard(scan.pixelMotor, 0);
      }
      movePixel(1);

      readColor(&temp.red, &temp.green, &temp.blue);
      // delay(480);//for night test

      //To make sure data is written in right sequence in the array
      if(scan.isSShape && i%2 != 0) {
        //Reverse sequence
        lineTemp[scan.pixelsPerLine-1-j][0] = min(temp.blue, 255);//minus 1 because j start from 0
        lineTemp[scan.pixelsPerLine-1-j][1] = min(temp.green, 255);
        lineTemp[scan.pixelsPerLine-1-j][2] = min(temp.red, 255);
        tft.drawPixel(temp.xCursor % 320, (scan.pixelsPerLine - 1 - temp.yCursor), tft.color565(min(temp.red, 255), min(temp.green, 255), min(temp.blue, 255)));
      } else {
        //Positive sequence
        lineTemp[j][0] = min(temp.blue, 255);
        lineTemp[j][1] = min(temp.green, 255);
        lineTemp[j][2] = min(temp.red, 255);
        tft.drawPixel(temp.xCursor % 320, temp.yCursor, tft.color565(min(temp.red, 255), min(temp.green, 255), min(temp.blue, 255)));//TODO:use xCursor and yCursor
      }
      SDWrite16(raw, temp.blue);
      SDWrite16(raw, temp.green);
      SDWrite16(raw, temp.red);
    }
    delay(100);

    printColor(temp.red, temp.green, temp.blue);

    for(uint16_t i = 0; i < scan.pixelsPerLine; ++i) {
      bmp.write(lineTemp[i][0]);
      bmp.write(lineTemp[i][1]);
      bmp.write(lineTemp[i][2]);
    }

    //Move back(Non S-Shape)
    if(scan.isSShape == false) {
      setMove(scan.pixelMotor, !scan.pixelDirection, 0);
      movePixel(scan.pixelsPerLine);
      delay(100);
    }

    setMove(scan.lineMotor, scan.lineDirection, 50);
    movePixel(1);
  }
  if(scan.totalLines%2 != 0) {
    setMove(scan.pixelMotor, !scan.pixelDirection, 0);
    movePixel(scan.pixelsPerLine);
  }
  setMove(scan.lineMotor, !scan.lineDirection, 50);
  movePixel(scan.totalLines);
  disableTimer();

  bmp.close();
  raw.close();
}

void enableTimer() {
  digitalWrite(MOTOR_HORIZONTAL_ENA, LOW);
  digitalWrite(MOTOR_VERTICAL_ENA, LOW);
  //only for init, not in use
  Timer1.initialize(motor[scan.pixelMotor].intervalTime);
  Serial.println("Timer1.init:"+(String)motor[scan.pixelMotor].intervalTime);
  timer.intervalTime = motor[scan.pixelMotor].intervalTime;
  timer.pulsePerPixel = timer.count = motor[scan.pixelMotor].pulsePerPixel;

  Timer1.attachInterrupt(timerInterrupt);
}

void disableTimer() {
  while(timer.count < timer.pulsePerPixel) {}
  Timer1.detachInterrupt();
  digitalWrite(MOTOR_HORIZONTAL_ENA, HIGH);
  digitalWrite(MOTOR_VERTICAL_ENA, HIGH);
}



//Sensor
void readColor(uint16_t* red, uint16_t* green, uint16_t* blue) {
  *red = tcs.read16(TCS34725_RDATAL);
  *green = tcs.read16(TCS34725_GDATAL);
  *blue = tcs.read16(TCS34725_BDATAL);
}

//Motor
void motorSetup() {
  //Pins configuraion
  pinMode(MOTOR_HORIZONTAL_DIR, OUTPUT);
  pinMode(MOTOR_HORIZONTAL_PUL, OUTPUT);
  pinMode(MOTOR_HORIZONTAL_ENA, OUTPUT);
  pinMode(MOTOR_VERTICAL_DIR, OUTPUT);
  pinMode(MOTOR_VERTICAL_PUL, OUTPUT);
  pinMode(MOTOR_VERTICAL_ENA, OUTPUT);
  digitalWrite(MOTOR_HORIZONTAL_ENA, HIGH);
  digitalWrite(MOTOR_VERTICAL_ENA, HIGH);

  //default
  motor[MOTOR_HORIZONTAL].intervalTime = HORIZONTAL_INTERVAL_TIME;
  motor[MOTOR_VERTICAL].intervalTime = VERTICAL_INTERVAL_TIME;
  motor[MOTOR_HORIZONTAL].pulsePerPixel = HORIZONTAL_PULSE_PER_PIXEL;
  motor[MOTOR_VERTICAL].pulsePerPixel = VERTICAL_PULSE_PER_PIXEL;
}

//deprecated
void move(uint8_t which, bool dir, uint64_t totalPulses) {

  if (which == MOTOR_HORIZONTAL) {            //which: '0' means horizontal, '1' means vertical
    digitalWrite(MOTOR_HORIZONTAL_DIR, dir);  //dir: 1 = HIGH, 0 = LOW (in Arduino.h)
    while (totalPulses > 0) {
      digitalWrite(MOTOR_HORIZONTAL_PUL, HIGH);
      digitalWrite(MOTOR_HORIZONTAL_PUL, LOW);
      delayMicroseconds(motor[0].intervalTime);
      totalPulses--;
    }
  } else {
    digitalWrite(MOTOR_VERTICAL_DIR, dir);  //dir: 1 = HIGH, 0 = LOW (in Arduino.h)
    while (totalPulses > 0) {
      digitalWrite(MOTOR_VERTICAL_PUL, HIGH);
      digitalWrite(MOTOR_VERTICAL_PUL, LOW);
      delayMicroseconds(motor[1].intervalTime);
      totalPulses--;
    }
  }

}

void setMove(uint8_t which, bool dir, uint8_t retard) {
  while(timer.count < timer.pulsePerPixel) {}

  timer.pin = (which == MOTOR_HORIZONTAL ? MOTOR_HORIZONTAL_PUL : MOTOR_VERTICAL_PUL);
  digitalWrite((which == MOTOR_HORIZONTAL ? MOTOR_HORIZONTAL_DIR : MOTOR_VERTICAL_DIR), dir);
  timer.pulsePerPixel = timer.count = motor[which].pulsePerPixel;

  timer.intervalTime = motor[which].intervalTime + retard;
  Timer1.setPeriod(timer.intervalTime);
}

void setMoveRetard(uint8_t which, uint8_t retard) {
  timer.intervalTime = motor[which].intervalTime + retard;
  Timer1.setPeriod(timer.intervalTime);
}

void movePixel(uint64_t pixels) {
  while(pixels > 0) {
    while(timer.count < timer.pulsePerPixel) {}
    timer.count = 0;
    --pixels;
  }
}

void timerInterrupt() {
  if (timer.count >= timer.pulsePerPixel) { return; }
  digitalWrite(timer.pin, HIGH);
  //TODO:maybe add some delay?
  digitalWrite(timer.pin, LOW);
  ++timer.count;
}

//Data
void saveData(uint64_t addr, uint8_t red, uint8_t green, uint8_t blue) {
  vti.write(3 * addr, red);
  vti.write(3 * addr + 1, green);
  vti.write(3 * addr + 2, blue);
}

//Little-endian ordering
void SDWrite16(File file, uint16_t data) {
  file.write(data % 256);//TODO:try | 0xFF
  file.write(floor(data / 256));//TODO:try >>8
}

//Interaction
long getPCInt() {
  while (!Serial.available()) {}  //Wait keyboard type in
  delay(100);                     //Wait all the data come in, otherwise there will be problem when clearing buffer
  long input = Serial.parseInt();
  while (Serial.available()) { Serial.read(); }  //Clear serial buffer
  return input;
}

//TODO:add acceptable range
long getPCInt(String message) {
  Serial.print(message);
  return getPCInt();
}

//Display
void printColor(int red, int green, int blue) {
  Serial.print("R"); Serial.print(red, DEC); Serial.print(" ");
  Serial.print("G"); Serial.print(green, DEC); Serial.print(" ");
  Serial.print("B"); Serial.print(blue, DEC); Serial.println(" ");
}