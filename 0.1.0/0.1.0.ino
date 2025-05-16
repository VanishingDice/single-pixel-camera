/*


Motor:
Power: 14.3V
Horizontal: 1A, 4 Microstep
Vertical: 0.5A, 4 Microstep

*/
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <SD.h>
#include <TFT_eSPI.h>
#include <Adafruit_TCS34725.h>
#include <VTI7064.h>
#include <TimerOne.h>//TODO: add timer move

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
struct StepperMotor{
  uint16_t pulsePerPixel;
  uint16_t intervalTime;
  uint8_t millimeterPerPulse;
};

struct{
  struct StepperMotor horizontal;
  struct StepperMotor vertical;
}motor;

struct{
  uint8_t pin;
  volatile uint16_t count = 0;
  uint16_t pulsePerPixel;
  uint16_t intervalTime;
}timer;

//Scan data, constantly in the scanning process
struct Scan{
  uint8_t path;
  uint16_t numberOfLines;//For movement
  uint16_t pixelsPerLine;
  uint16_t xPixels;//For image
  uint16_t yPixels;
  uint32_t filename;
}scan;

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

//Data for temp using, various data when scanning
struct Temp{
  uint16_t xCursor;//Coordinate of image
  uint16_t yCursor;
  uint16_t red;//Color data
  uint16_t green;
  uint16_t blue;
}temp;

//Other devices
File bmp;
LiquidCrystal_I2C lcd(0x27,16,2);
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS,TCS34725_GAIN_16X);
TFT_eSPI tft = TFT_eSPI();
VTI7064 vti = VTI7064(9);

//Static var
int bmpHead[54] = {
  0x42,0x4d,//BM word
  0x00,0x00,0x00,0x00,//File size
  0x00,0x00,0x00,0x00,//Reserve
  0x36,0x00,0x00,0x00,//File start
  0x28,0x00,0x00,0x00,//DIB header size
  0x64,0x00,0x00,0x00,//Image width(100)(18-21)
  0x64,0x00,0x00,0x00,//Image height(100)(22-25)
  0x01,0x00,//Number of color plane(must be 1)
  0x20,0x00,//How many bits for one pixel(32)
  0x00,0x00,0x00,0x00,//Zip method(0 for none)
  0x00,0x00,0x00,0x00,//Raw image size
  0x13,0x0b,0x00,0x00,0x13,0x0b,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00//Other
};//Head data for BMP file

//temp var
uint16_t i;

int motorIntervalTimeShift = 0;


void setup() {
  Serial.begin(9600);
  Serial.println("Serial initialization done.");

  motorSetup();
  Serial.println("Motor initialization done.");

  lcd.begin(16,2);
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Hello!");
  Serial.println("LCD initialization done.");

  if (!SD.begin(53)) {
    Serial.println("SD initialization failed!");
    while(1){}
  }
  Serial.println("SD initialization done.");

  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);
  // tft.fillScreen(TFT_WHITE);
  Serial.println("TFT initialization done.");

  if(!vti.begin()) {
    Serial.println("SRAM initialization failed!");
    // while(1){}
  } else {
    Serial.println("SRAM initialization done.");
  }

  if(!tcs.begin()) {
    Serial.println("Color sensor initialization failed!");
  } else {
    Serial.println("Color sensor initialization done.");
  } 

}

void loop() {
  // motorIntervalTimeShift = getPCInt("Enter motor interval time shift:\n");
  // motor.horizontal.intervalTime = 280 - motorIntervalTimeShift;
  // motor.vertical.intervalTime = 280 - motorIntervalTimeShift;
  // Serial.println(motor.horizontal.intervalTime);

  uint32_t filename = getPCInt("Last filename is: "+String(scan.filename)+". \nEnter filename:") % 100000000;
  Serial.println(filename);
  uint8_t path = getPCInt("Enter path(0-7): ");
  Serial.println(pathComment[path]);
  uint16_t xPixels = getPCInt("Enter xPixels:");
  Serial.println(xPixels);
  uint16_t yPixels = getPCInt("Enter yPixels:");
  Serial.println(yPixels);

  setScan(path, xPixels, yPixels, filename);
  scanTask();

}

//Mainly functions
void setScan(uint8_t path, uint16_t xPixels, uint16_t yPixels, uint32_t filename) {
  scan.path = path;
  scan.filename = filename;
  scan.xPixels = xPixels;
  scan.yPixels = yPixels;
  switch(scan.path) {
    case 0:
    case 1:
    case 2:
    case 3:
      scan.pixelsPerLine = xPixels;
      scan.numberOfLines = yPixels;
      break;
    case 4:
    case 5:
    case 6:
    case 7:
      scan.pixelsPerLine = yPixels;
      scan.numberOfLines = xPixels;
  }
}

void scanTask() {
  //Real direction
  // bmpHead[18] = scan.yPixels%256;
  // bmpHead[19] = floor(scan.yPixels/256);
  // bmpHead[22] = scan.xPixels%256;
  // bmpHead[23] = floor(scan.xPixels/256);
  //For quicker test
  bmpHead[18] = scan.pixelsPerLine%256;
  bmpHead[19] = floor(scan.pixelsPerLine/256);
  bmpHead[22] = scan.numberOfLines%256;
  bmpHead[23] = floor(scan.numberOfLines/256);

  String dir = String(scan.filename);
  dir.concat(".bmp");
  bmp = SD.open(dir,FILE_WRITE);
  for(int i = 0; i < 54; i++) {
    bmp.write(bmpHead[i]);
  }

  for(uint16_t i = 0; i < scan.numberOfLines; ++i) {
    enableTimerMove();
    for(uint16_t j = 0; j < scan.pixelsPerLine; ++j) {
      readColor(&temp.red, &temp.green, &temp.blue);
      // Serial.print("R: "); Serial.print(temp.red, DEC); Serial.print(" ");
      // Serial.print("G: "); Serial.print(temp.green, DEC); Serial.print(" ");
      // Serial.print("B: "); Serial.print(temp.blue, DEC); Serial.println(" ");

      bmp.write(min(temp.blue,255));
      bmp.write(min(temp.green,255));
      bmp.write(min(temp.red,255));
      bmp.write(0xFF);

      tft.drawPixel(temp.xCursor, temp.yCursor, tft.color565(min(temp.red,255), min(temp.green,255), min(temp.blue,255)));

      // saveData(temp.yCursor*scan.yPixels+temp.xCursor, temp.red, temp.green, temp.blue);
      // displayData();
      
      // nextPixel();
      timerNextPixel();
    }
    disableTimerMove();
    // delay(1000);
    // motor.horizontal.intervalTime += motorIntervalTimeShift;
    // motor.vertical.intervalTime += motorIntervalTimeShift;
    pixelsBack();
    // motor.horizontal.intervalTime -= motorIntervalTimeShift;
    // motor.vertical.intervalTime -= motorIntervalTimeShift;
    nextLine();
  }
  // motor.horizontal.intervalTime += motorIntervalTimeShift;
  // motor.vertical.intervalTime += motorIntervalTimeShift;
  linesBack();
  // motor.horizontal.intervalTime -= motorIntervalTimeShift;
  // motor.vertical.intervalTime -= motorIntervalTimeShift;
  bmp.close();

}

void nextPixel() {
  switch(scan.path) {
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
  switch(scan.path) {
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
  switch(scan.path) {
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
  switch(scan.path) {
    case 0:
    case 1:
      movePixel(MOTOR_VERTICAL, MOTOR_BACKWARD, scan.numberOfLines);
      temp.yCursor -= scan.numberOfLines;
      break;
    case 2:
    case 3:
      movePixel(MOTOR_VERTICAL, MOTOR_FORWARD, scan.numberOfLines);
      temp.yCursor += scan.numberOfLines;
      break;
    case 4:
    case 5:
      movePixel(MOTOR_HORIZONTAL, MOTOR_BACKWARD, scan.numberOfLines);
      temp.xCursor -= scan.numberOfLines;
      break;
    case 6:
    case 7:
      movePixel(MOTOR_HORIZONTAL, MOTOR_FORWARD, scan.numberOfLines);
      temp.xCursor += scan.numberOfLines;
      break;
  }
}

void enableTimerMove() {
  digitalWrite(MOTOR_HORIZONTAL_ENA, LOW);
  digitalWrite(MOTOR_VERTICAL_ENA, LOW);
  switch(scan.path) {
    case 0:
    case 2:
      digitalWrite(MOTOR_HORIZONTAL_DIR, MOTOR_FORWARD);
      Timer1.initialize(motor.horizontal.intervalTime);
      timer.pulsePerPixel = timer.count = motor.horizontal.pulsePerPixel;
      timer.intervalTime = motor.horizontal.intervalTime;
      break;
    case 1:
    case 3:
      digitalWrite(MOTOR_HORIZONTAL_DIR, MOTOR_BACKWARD);
      Timer1.initialize(motor.horizontal.intervalTime);
      timer.pulsePerPixel = timer.count = motor.horizontal.pulsePerPixel;
      timer.intervalTime = motor.horizontal.intervalTime;
      break;
    case 4:
    case 6:
      digitalWrite(MOTOR_VERTICAL_DIR, MOTOR_FORWARD);
      Timer1.initialize(motor.vertical.intervalTime);
      timer.pulsePerPixel = timer.count = motor.vertical.pulsePerPixel;
      timer.intervalTime = motor.vertical.intervalTime;
      break;
    case 5:
    case 7:
      digitalWrite(MOTOR_VERTICAL_DIR, MOTOR_BACKWARD);
      Timer1.initialize(motor.vertical.intervalTime);
      timer.pulsePerPixel = timer.count = motor.vertical.pulsePerPixel;
      timer.intervalTime = motor.vertical.intervalTime;
      break;
  }
  Timer1.attachInterrupt(timerInterrupt);
}

void disableTimerMove() {
  while(timer.count < timer.pulsePerPixel) {}
  Timer1.detachInterrupt();
  digitalWrite(MOTOR_HORIZONTAL_ENA, HIGH);
  digitalWrite(MOTOR_VERTICAL_ENA, HIGH);
}

void timerNextPixel() {
  while(timer.count < timer.pulsePerPixel) {}
  timer.count = 0;
}

void displayData() {

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
  motor.horizontal.intervalTime = 280;
  motor.vertical.intervalTime = 280;
  motor.horizontal.pulsePerPixel = 10;
  motor.vertical.pulsePerPixel = 10;
}

void move(uint8_t which, bool dir, uint64_t count) {
  digitalWrite(MOTOR_HORIZONTAL_ENA, LOW);
  digitalWrite(MOTOR_VERTICAL_ENA, LOW);

  if(which == MOTOR_HORIZONTAL) {//which: '0' means horizontal, '1' means vertical
    digitalWrite(MOTOR_HORIZONTAL_DIR, dir);//dir: 1 = HIGH, 0 = LOW (in Arduino.h)
    while (count > 0) {
      digitalWrite(MOTOR_HORIZONTAL_PUL, HIGH);
      digitalWrite(MOTOR_HORIZONTAL_PUL, LOW);
      delayMicroseconds(motor.horizontal.intervalTime);
      count--;
    }
  } else {
    digitalWrite(MOTOR_VERTICAL_DIR, dir);//dir: 1 = HIGH, 0 = LOW (in Arduino.h)
    while (count > 0) {
      digitalWrite(MOTOR_VERTICAL_PUL, HIGH);
      digitalWrite(MOTOR_VERTICAL_PUL, LOW);
      delayMicroseconds(motor.vertical.intervalTime);
      count--;
    }
  }

  digitalWrite(MOTOR_HORIZONTAL_ENA, HIGH);
  digitalWrite(MOTOR_VERTICAL_ENA, HIGH);
}

void movePixel(uint8_t which, bool dir, uint64_t pixels) {
  move(which, dir, pixels*(which==MOTOR_HORIZONTAL?motor.horizontal.pulsePerPixel:motor.vertical.pulsePerPixel));
}

void timerInterrupt() {
  if(timer.count >= timer.pulsePerPixel){return;}
  digitalWrite(timer.pin, HIGH);
  digitalWrite(timer.pin, LOW);
  ++timer.count;
}

//Data
void saveData(uint64_t addr, uint8_t red, uint8_t green, uint8_t blue) {
  vti.write(3*addr, red);
  vti.write(3*addr+1, green);
  vti.write(3*addr+2, blue);
}

//Interaction
long getPCInt() {
  while(!Serial.available()){}//Wait keyboard type in
  delay(100);//Wait all the data come in, otherwise there will be problem when clearing buffer
  long input = Serial.parseInt();
  while(Serial.available()){Serial.read();}//Clear serial buffer
  return input;
}

long getPCInt(String message) {
  Serial.print(message);
  return getPCInt();
}

//Display
