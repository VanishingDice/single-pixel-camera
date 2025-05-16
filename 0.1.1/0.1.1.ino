/*
Motor:
Power: 12V
Horizontal: 1.5A, 4 Microstep
Vertical: 1.5A, 4 Microstep

TODO:movespeed by Timer1.setPeriod(microseconds);

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
  0x20, 0x00,                                                                                     //How many bits for one pixel(32) TODO:change to 24bits
  0x00, 0x00, 0x00, 0x00,                                                                         //Zip method(0 for none)
  0x00, 0x00, 0x00, 0x00,                                                                         //Raw image size
  0x13, 0x0b, 0x00, 0x00, 0x13, 0x0b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00  //Other
};                                                                                                //Head data for BMP file

//temp var
uint16_t i;

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
  if (!SD.begin(53)) {
    Serial.println("SD initialization failed!");
    while (1) {}
  }
  Serial.println("SD initialization done.");

  scan.filename = getPCInt("Last filename is: " + String(scan.filename) + ". \nEnter filename:") % 100000000;//TODO:read filename from sd card
  Serial.println(scan.filename);
  scan.path = getPCInt("Enter path(0-7): ");
  Serial.println(pathComment[scan.path]);
  scan.pixelsPerLine = getPCInt("Enter pixelsPerLine:");
  Serial.println(scan.pixelsPerLine);
  scan.totalLines = getPCInt("Enter totalLines:");
  Serial.println(scan.totalLines);
  setScanMotor();

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

  enableTimer();
  for (uint16_t i = 0; i < scan.totalLines; ++i) {

    lcd.setCursor(0, 0);
    lcd.print(i+"               ");
    setMove(scan.pixelMotor,scan.pixelDirection);
    for (uint16_t j = 0; j < scan.pixelsPerLine; ++j) {
      movePixel(1);

      readColor(&temp.red, &temp.green, &temp.blue);
      // printColor(temp.red, temp.green, temp.blue);

      bmp.write(min(temp.blue, 255));
      bmp.write(min(temp.green, 255));
      bmp.write(min(temp.red, 255));
      bmp.write(0xFF);//TODO:change to 24bits

      tft.drawPixel(temp.xCursor, temp.yCursor, tft.color565(min(temp.red, 255), min(temp.green, 255), min(temp.blue, 255)));//TODO:use xCursor and yCursor

      // saveData(temp.yCursor*scan.yPixels+temp.xCursor, temp.red, temp.green, temp.blue);
      // displayData();
    }
    delay(1000);
    setMove(scan.pixelMotor, !scan.pixelDirection);
    movePixel(scan.pixelsPerLine);
    delay(1000);

    setMove(scan.lineMotor, scan.lineDirection);
    movePixel(1);
  }
  setMove(scan.lineMotor, !scan.lineDirection);
  movePixel(scan.totalLines);
  disableTimer();

  bmp.close();
}

/*
void nextPixel() {
  switch (scan.path) {
    case 0:
    case 2:
      movePixel(MOTOR_HORIZONTAL, MOTOR_FORWARD, 1);
      ++temp.xCursor;
      break;
    case 1:
    case 3:
      movePixel(MOTOR_HORIZONTAL, MOTOR_BACKWARD, 1);
      --temp.xCursor;
      break;
    case 4:
    case 6:
      movePixel(MOTOR_VERTICAL, MOTOR_FORWARD, 1);
      ++temp.yCursor;
      break;
    case 5:
    case 7:
      movePixel(MOTOR_VERTICAL, MOTOR_BACKWARD, 1);
      --temp.yCursor;
      break;
  }
}

void pixelsBack() {
  //Pixels back
  switch (scan.path) {
    case 0:
    case 2:
      movePixel(MOTOR_HORIZONTAL, MOTOR_BACKWARD, scan.pixelsPerLine);
      temp.xCursor -= scan.pixelsPerLine;
      break;
    case 1:
    case 3:
      movePixel(MOTOR_HORIZONTAL, MOTOR_FORWARD, scan.pixelsPerLine);
      temp.xCursor += scan.pixelsPerLine;
      break;
    case 4:
    case 6:
      movePixel(MOTOR_VERTICAL, MOTOR_BACKWARD, scan.pixelsPerLine);
      temp.yCursor -= scan.pixelsPerLine;
      break;
    case 5:
    case 7:
      movePixel(MOTOR_VERTICAL, MOTOR_FORWARD, scan.pixelsPerLine);
      temp.yCursor += scan.pixelsPerLine;
      break;
  }
}

void nextLine() {
  //Next line
  switch (scan.path) {
    case 0:
    case 1:
      movePixel(MOTOR_VERTICAL, MOTOR_FORWARD, 1);
      ++temp.yCursor;
      break;
    case 2:
    case 3:
      movePixel(MOTOR_VERTICAL, MOTOR_BACKWARD, 1);
      --temp.yCursor;
      break;
    case 4:
    case 5:
      movePixel(MOTOR_HORIZONTAL, MOTOR_FORWARD, 1);
      ++temp.xCursor;
      break;
    case 6:
    case 7:
      movePixel(MOTOR_HORIZONTAL, MOTOR_BACKWARD, 1);
      --temp.xCursor;
      break;
  }
}

void linesBack() {
  //Line back
  switch (scan.path) {
    case 0:
    case 1:
      movePixel(MOTOR_VERTICAL, MOTOR_BACKWARD, scan.totalLines);
      temp.yCursor -= scan.totalLines;
      break;
    case 2:
    case 3:
      movePixel(MOTOR_VERTICAL, MOTOR_FORWARD, scan.totalLines);
      temp.yCursor += scan.totalLines;
      break;
    case 4:
    case 5:
      movePixel(MOTOR_HORIZONTAL, MOTOR_BACKWARD, scan.totalLines);
      temp.xCursor -= scan.totalLines;
      break;
    case 6:
    case 7:
      movePixel(MOTOR_HORIZONTAL, MOTOR_FORWARD, scan.totalLines);
      temp.xCursor += scan.totalLines;
      break;
  }
}
*/

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

void printColor(int red, int green, int blue) {
  Serial.print("R: "); Serial.print(red, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(green, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.print(blue, DEC); Serial.println(" ");
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
  motor[MOTOR_HORIZONTAL].intervalTime = 370;
  motor[MOTOR_VERTICAL].intervalTime = 370;
  motor[MOTOR_HORIZONTAL].pulsePerPixel = 10;
  motor[MOTOR_VERTICAL].pulsePerPixel = 10;
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

void setMove(uint8_t which, bool dir) {
  timer.pin = (which == MOTOR_HORIZONTAL ? MOTOR_HORIZONTAL_PUL : MOTOR_VERTICAL_PUL);
  digitalWrite((which == MOTOR_HORIZONTAL ? MOTOR_HORIZONTAL_DIR : MOTOR_VERTICAL_DIR), dir);
  timer.pulsePerPixel = timer.count = motor[which].pulsePerPixel;
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
