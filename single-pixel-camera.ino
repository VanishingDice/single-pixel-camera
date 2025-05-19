/*
Motor:
Power: 12V
Horizontal: 1.5A, 4 Microstep
Vertical: 1.5A, 4 Microstep

V1.0.0:add touch UI
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
#include <ResistiveTouchScreen.h>
#include <DS3231.h>

//Pin definition
#define MOTOR_HORIZONTAL 0
#define MOTOR_VERTICAL 1
#define MOTOR_FORWARD 1//H-->end V-->motor
#define MOTOR_BACKWARD 0//H-->motor V-->end
#define MOTOR_HORIZONTAL_PUL 2
#define MOTOR_HORIZONTAL_DIR 3
#define MOTOR_HORIZONTAL_ENA 4
#define MOTOR_VERTICAL_PUL 5
#define MOTOR_VERTICAL_DIR 6
#define MOTOR_VERTICAL_ENA 7
//Move parameters
#define HORIZONTAL_INTERVAL_TIME 320
#define VERTICAL_INTERVAL_TIME 320
#define HORIZONTAL_PULSE_PER_PIXEL 80 //Minimum 80 for 1204 and TCS34725, 4 microstep
#define VERTICAL_PULSE_PER_PIXEL 80

// #define THEME_TIME TFT_SKYBLUE
#define THEME_ORG tft.color565(255, 180, 80)
#define THEME_GRN tft.color565(90, 240, 150)

//For computer control
//#define CAMERA_DEBUG_MODE

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

enum Sensor {
  TCS34725,
  AS73211,
  AS7341, 
  SENSOR_ALL
};

//Scan data, constantly in the scanning process
struct ScanParam {
  uint8_t path = 0;
  uint8_t currentPoint = 1;//0:left up, 1:right up, 2:right down, 3:left down
  // uint8_t startPoint = 2;//TODO:REMOVE
  uint16_t totalLines = 100;  //For movement
  uint16_t pixelsPerLine = 100;

  uint8_t pixelMotor = MOTOR_HORIZONTAL;
  uint8_t pixelDirection = MOTOR_FORWARD;
  uint8_t lineMotor = MOTOR_VERTICAL;
  uint8_t lineDirection = MOTOR_FORWARD;

  uint32_t filename = 0;

  bool isSShape = false;

  Sensor sensor = TCS34725;

  uint8_t integrationTime = TCS34725_INTEGRATIONTIME_24MS; //Maximum interval*pulsePerPixel
  tcs34725Gain_t gain = TCS34725_GAIN_60X;
  //TODO:add more sensors
} scan;

struct Button{
  uint16_t x;
  uint16_t y;
  uint16_t w;
  uint16_t h;
  String name;
  uint16_t color;
};

//Comment text for path
String pathComment[8] = {
  "Left, then down",
  "Right, then down",
  "Left, then up",
  "Right, then up",
  "Down, then Left",
  "Up, then Left",
  "Down, then Right",
  "Up, then Right"
};

//For output gain information
const char* GainStr[] = {"GAIN_1X","GAIN_4X","GAIN_16X","GAIN_60X"};

const char* SensorStr[] = {"TCS34725","AS73211","AS7341"};

//
struct {
  uint16_t x;
  uint16_t y;
  uint16_t color;
  uint16_t background;
  uint16_t size;
  uint8_t textWidth;
} drawTimeFormat;

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
TFT_eSPI tft = TFT_eSPI();//needs to be 4-wire mode, but don't connect MISO
VTI7064 vti = VTI7064(9);
ResistiveTouchScreen touch = ResistiveTouchScreen();
RTClib rtc;
DateTime now;
DateTime prevTime;

//Static var
static int bmpHead[54] = {
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


void setup() {
  Serial.begin(115200);
  Serial.println("Serial initialization done.");

  motorSetup();
  Serial.println("Motor initialization done.");

  lcd.begin(16, 2);
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Hello!");
  Serial.println("LCD initialization done.");

  //#EN of TXB0108
  pinMode(26, OUTPUT);
  digitalWrite(26, HIGH);
  //Card detect
  pinMode(24, INPUT_PULLUP);
  if(digitalRead(24) == HIGH) {
    Serial.println("Waiting card...");
  }
  while(digitalRead(24) == HIGH) {}
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

  setDrawTimeFormat(9, 0, TFT_SKYBLUE, TFT_BLACK, 2);
  prevTime = now = rtc.now();

}

void loop() {
  mainScreen();
  // Serial.println(String(getIntegerScreen("Test:")));

  // computerControl();
}

//UI--------------------
//Main
void mainScreen() {
  struct Button buttons[4] = {
    {4, 164, 150, 150, "Scan", THEME_GRN},
    {164, 164, 150, 150, "Setting", TFT_SILVER},
    {4, 324, 150, 150, "Move", THEME_ORG},
    {164, 324, 150, 150, "Image", TFT_SKYBLUE},
  };

  uint16_t background = tft.color565(50, 50, 50);
  bool redraw = false;

  while(1) {
    drawMainScreen(buttons, 4, background);
    setDrawTimeFormat(9, 0, TFT_SKYBLUE, background, 2);
    drawTime();
    redraw = false;
    while(!redraw) {
      refreshTime();
      switch(getButtonPressed(buttons, 4)) {
        case 0:
          scanScreen();
          redraw = true;
          break;
        case 1:
          settingScreen();
          redraw = true;
          break;
        case 2:
          moveScreen();
          redraw = true;
          break;
        case 3:
          imageScreen();
          redraw = true;
          break;
      }
    }
    // scan.totalLines = getPCInt("Enter totalLines:");
    // Serial.println(scan.totalLines);
  }
}
void drawMainScreen(Button* buttons, int size, uint16_t background) {
  tft.fillScreen(background);
  drawParam(background);
  drawButtons(buttons, size, background);
}

//Scan
void scanScreen() {
  struct Button buttons[3] = {
    {4, 164, 150, 150, "Test", THEME_GRN},
    {4, 324, 150, 150, "Exit", THEME_ORG},
    {164, 324, 150, 150, "Start", TFT_SKYBLUE},
  };

  uint16_t background = tft.color565(200, 255, 200);
  bool redraw = false;

  while(1) {
    drawScanScreen(buttons, 3, background);
    setDrawTimeFormat(9, 0, TFT_SKYBLUE, background, 2);
    drawTime();
    redraw = false;
    while(!redraw) {
      refreshTime();
      switch(getButtonPressed(buttons, 3)) {
        case 0:
          scanPreview();
          break;
        case 1:
          return;
          break;
        case 2:
          scanTask();
          // redraw = true;
          return;
          break;
      }
    }
  }
}
void drawScanScreen(Button* buttons, int size, uint16_t background) {
  tft.fillScreen(background);
  drawParam(background);
  drawButtons(buttons, size, background);
}

//Setting
void settingScreen() {
  struct Button buttons[6] =  {
    {4, 164, 150, 70, "Scan Area", TFT_DARKGREY},
    // {164, 164, 150, 70, "Scan Path", TFT_DARKGREY},
    {4, 244, 150, 70, "Filename", TFT_DARKGREY},
    {164, 244, 150, 70, "Storage", TFT_DARKGREY},
    {4, 324, 150, 70, "Motor", TFT_DARKGREY},
    {164, 324, 150, 70, "Sensor", TFT_DARKGREY},
    {4, 404, 150, 70, "Exit", TFT_DARKGREY},
  };

  uint16_t background = tft.color565(80, 80, 80);
  bool redraw = false;
  int pressed = -1;

  while(1) {
    drawSettingScreen(buttons, 6, background);
    setDrawTimeFormat(9, 0, TFT_SKYBLUE, background, 2);
    drawTime();
    redraw = false;
    while(!redraw) {
      refreshTime();
      pressed = getButtonPressed(buttons, 6);
      switch(pressed) {
        case 0:
          scanAreaScreen();
          redraw = true;
          break;
        case 1:
          // 
          break;
        case 2:
          // 
          break;
        case 3:
          // 
          break;
        case 4:
          sensorScreen();
          redraw = true; 
          break;
        case 5:
          return;
          break;
      }
    }
  }
}
void drawSettingScreen(Button* buttons, int size, uint16_t background) {
  tft.fillScreen(background);
  drawParam(background);
  drawButtons(buttons, size, background);
}

//TODO:
void moveScreen() {
  struct Button buttons[] = {
    {4, 164, 150, 150, "Scan", THEME_GRN},
  };

  uint16_t background = tft.color565(50, 50, 50);

  // drawScreen(buttons, 0 , background);
  setDrawTimeFormat(9, 0, TFT_SKYBLUE, background, 2);
  drawTime();

  bool exit = false;
  while(1) {
    refreshTime();
    switch(getButtonPressed(buttons, 0)) {
      case 0:
        ;
        exit = true;
        break;
      case 1:
        ;
        exit = true;
        break;
      case 2:
        ;
        exit = true;
        break;
      case 3:
        ;
        exit = true;
        break;
    }
    if(exit) {
      break;
    }
  }
}

//TODO:
void imageScreen() {
  tft.fillScreen(TFT_SKYBLUE);
}

//Scan Area Setting
void scanAreaScreen() {
  struct Button buttons[11] = {
    {244, 84, 70, 70, "X", TFT_SKYBLUE},
    {244, 164, 70, 70, "Y", TFT_SKYBLUE},
    {244, 244, 70, 70, "Path", TFT_SKYBLUE},
    {4, 324, 150, 70, "S Shape", TFT_SKYBLUE},
    {164, 324, 150, 70, "Pulse/Pixel", TFT_SKYBLUE},
    {4, 404, 150, 70, "Exit", TFT_SKYBLUE},
    {164, 404, 150, 70, "Save", TFT_SKYBLUE},
    //for current point button, no display
    {9, 109, 75, 75, "0", TFT_BLACK},//Left up
    {84, 109, 75, 75, "1", TFT_BLACK},//Right up
    {84, 184, 75, 75, "2", TFT_BLACK},//Right down
    {9, 184, 75, 75, "3", TFT_BLACK},//Left down

  };

  uint16_t background = tft.color565(100, 100, 100);

  uint16_t xTmp, yTmp;
  uint8_t pathTmp = scan.path;
  getScanXY(&xTmp, &yTmp);
  uint16_t pppTmp = motor[scan.pixelMotor].pulsePerPixel;
  bool sShapeTmp = scan.isSShape;
  bool redraw = false;//whole screen refresh
  uint8_t currentPointTmp = scan.currentPoint;

  while(1) {
    drawScanAreaScreen(buttons, 7, background, xTmp, yTmp, pppTmp, pathTmp, sShapeTmp, currentPointTmp);
    setDrawTimeFormat(9, 0, TFT_SKYBLUE, background, 2);
    drawTime();
    redraw = false;
    while(!redraw) {
      refreshTime();
      switch(getButtonPressed(buttons, 11)) {
        case 0:
          xTmp = getIntegerScreen("X:");
          redraw = true;
          break;
        case 1:
          yTmp = getIntegerScreen("Y:");
          redraw = true;
          break;
        case 2:
          pathTmp += 1;
          if(pathTmp >= 8) {
            pathTmp = 0;
          }
          freshScanAreaArrowMatrix(background, xTmp, yTmp, pathTmp);
          break;
        case 3:
          sShapeTmp = !sShapeTmp;
          freshScanAreaSShape(sShapeTmp, background);
          break;
        case 4:
          pppTmp = getIntegerScreen(F("Pulse Per Pixel:"));
          redraw = true;
          break;
        case 5:
          return;
          break;
        case 6:
          setScanXY(xTmp, yTmp);
          scan.path = pathTmp;
          motor[scan.pixelMotor].pulsePerPixel = pppTmp;
          motor[scan.lineMotor].pulsePerPixel = pppTmp;
          scan.isSShape = sShapeTmp;
          scan.currentPoint = currentPointTmp;
          return;
          break;
        case 7:
          currentPointTmp = 0;
          freshScanAreaCurrentPoint(currentPointTmp, xTmp, yTmp, background);
          break;
        case 8:
          currentPointTmp = 1;
          freshScanAreaCurrentPoint(currentPointTmp, xTmp, yTmp, background);
          break;
        case 9:
          currentPointTmp = 2;
          freshScanAreaCurrentPoint(currentPointTmp, xTmp, yTmp, background);
          break;
        case 10:
          currentPointTmp = 3;
          freshScanAreaCurrentPoint(currentPointTmp, xTmp, yTmp, background);
          break;
      }
    }
  }
}
void drawScanAreaScreen(Button* buttons, int size, uint16_t background, uint16_t x, uint16_t y, uint16_t pulsePerPixel, uint8_t path, bool isSShape, uint8_t currentPoint) {
  float ratio = (float)150 / max(x,y);
  tft.fillScreen(background);
  tft.drawRect(9, 109, ratio*x, ratio*y, getFrontColor(background));
  drawArrowMatrix(9, 109, ratio*x, ratio*y, path, getFrontColor(background), 15);
  tft.setTextColor(getFrontColor(background));
  tft.setTextSize(2);
  tft.setTextDatum(BC_DATUM);
  tft.drawString(String(x), 9+(ratio*x/2), 109-2);
  tft.setTextDatum(ML_DATUM);
  tft.drawString(String(y), 9+ratio*x+3, 109+(ratio*y/2));
  drawButtons(buttons, size, background);

  tft.setTextColor(getFrontColor(background));
  tft.setTextDatum(TL_DATUM);
  tft.drawString("Total time:", 9, 24);
  tft.setTextColor(TFT_SKYBLUE);
  // TODO:Not accurate
  long totalSeconds = ((long)x * y * pulsePerPixel * motor[scan.pixelMotor].intervalTime 
                          + y * pulsePerPixel * motor[scan.lineMotor].intervalTime) / 1000000 * (2 - scan.isSShape);//calculate based on motor movement time
  tft.drawString(String((int)floor(totalSeconds/3600))+"h "+String((int)floor(totalSeconds/60%60))+"m "+String(totalSeconds%60)+"s", 9, 44);

  if(isSShape) {
    tft.setTextColor(THEME_GRN);
    tft.drawString("S-Shape Enable", 9, 269);
  } else {
    tft.setTextColor(TFT_SILVER);
    tft.drawString("S-Shape Disable", 9, 269);
  }
  freshScanAreaCurrentPoint(currentPoint, x, y, background);

}
void freshScanAreaArrowMatrix(uint16_t background, uint16_t x, uint16_t y, uint8_t path) {
  float ratio = (float)150 / max(x,y);
  // tft.fillRect(10, 110, ratio*x - 2, ratio*y - 2, background);
  // Serial.println("X:"+String(ratio*x)+", Y:"+String(ratio*y)+", Path:"+String(path));
  drawArrowMatrix(9, 109, ratio*x, ratio*y, path-1<0 ? 7 : path-1, background, 15);//cover the last arrows
  drawArrowMatrix(9, 109, ratio*x, ratio*y, path, getFrontColor(background), 15);
}
void freshScanAreaSShape(bool isSShape, uint16_t background) {
  if(isSShape) {
    tft.setTextColor(THEME_GRN, background);
    tft.drawString("S-Shape Enable ", 9, 269);
  } else {
    tft.setTextColor(TFT_SILVER, background);
    tft.drawString("S-Shape Disable", 9, 269);
  }
}
void freshScanAreaCurrentPoint(uint8_t currentPoint, uint16_t x, uint16_t y, uint16_t background) {
  float ratio = (float)150 / max(x,y);

  for(int i = 0; i < 4; ++i) {
    tft.fillSmoothCircle((i == 0 || i == 3) ? 9 : 9 + x*ratio, i <= 1  ? 109 : 109 + y*ratio, 5, background, background);
  }
  tft.drawRect(9, 109, ratio*x, ratio*y, getFrontColor(background));
  tft.fillSmoothCircle((currentPoint == 0 || currentPoint == 3) ? 9 : 9 + x*ratio, currentPoint <= 1  ? 109 : 109 + y*ratio, 5, THEME_ORG, background);
}

//Sensor select screen
void sensorScreen() {
  struct Button buttons[6] =  {
    {4, 4, 150, 150, "TCS34725", TFT_DARKGREY},
    {164, 4, 150, 150, "AS73211", TFT_DARKGREY},
    {4, 164, 150, 150, "AS7341", TFT_DARKGREY},
    {164, 164, 150, 150, "Reserve", TFT_DARKGREY},
    {4, 324, 150, 150, "Exit", TFT_DARKGREY},
    {164, 324, 150, 150, "Save", TFT_DARKGREY},
  };

  uint16_t background = tft.color565(80, 80, 80);
  bool redraw = false;
  Sensor sensorTmp = scan.sensor;

  while(1) {
    for(int i = 0; i < 3; ++i) {
      buttons[i].color = TFT_DARKGREY;
    }
    buttons[sensorTmp].color = THEME_GRN;
    drawSensorScreen(buttons, 6, background);
    redraw = false;
    while(!redraw) {
      switch(getButtonPressed(buttons, 6)) {
        case 0:
          sensorTmp = TCS34725;
          redraw = true;
          break;
        case 1:
          sensorTmp = AS73211;
          redraw = true;
          break;
        case 2:
          sensorTmp = AS7341;
          redraw = true;
          break;
        case 3:
          // 
          break;
        case 4:
          return;
          break;
        case 5:
          scan.sensor = sensorTmp;
          return;
          break;
      }
    }
  }
}
void drawSensorScreen(Button* buttons, int size, uint16_t background) {
  tft.fillScreen(background);
  drawButtons(buttons, size, background);
}

//Get integer from touch screen, TODO:change the least bit variation bug
long getIntegerScreen(String comment) {
  struct Button buttons[12] = {
    {14, 164, 90, 70, "1", TFT_DARKGREY},
    {114, 164, 90, 70, "2", TFT_DARKGREY},
    {214, 164, 90, 70, "3", TFT_DARKGREY},
    {14, 244, 90, 70, "4", TFT_DARKGREY},
    {114, 244, 90, 70, "5", TFT_DARKGREY},
    {214, 244, 90, 70, "6", TFT_DARKGREY},
    {14, 324, 90, 70, "7", TFT_DARKGREY},
    {114, 324, 90, 70, "8", TFT_DARKGREY},
    {214, 324, 90, 70, "9", TFT_DARKGREY},
    {14, 404, 90, 70, "Back", TFT_DARKGREY},
    {114, 404, 90, 70, "0", TFT_DARKGREY},
    {214, 404, 90, 70, "Save", TFT_DARKGREY},
  };

  uint16_t background = tft.color565(180, 180, 190);
  uint16_t textBoxBackground = tft.color565(50, 50, 50);
  long tmp = 0;
  bool redraw = false;
  bool isPressed = false;

  while(1) {
    drawIntegerScreen(buttons, 12, background, comment, tmp, textBoxBackground);
    freshIntegerTextBox(tmp, textBoxBackground);
    redraw = false;
    while(!redraw) {
      freshIntegerCursor(tmp, textBoxBackground);
      // freshIntegerTextBox(tmp, textBoxBackground);
      switch(getButtonPressed(buttons, 12)) {
        case 0:
          tmp = tmp*10 + 1;
          isPressed = true;
          break;
        case 1:
          tmp = tmp*10 + 2;
          isPressed = true;
          break;
        case 2:
          tmp = tmp*10 + 3;
          isPressed = true;
          break;
        case 3:
          tmp = tmp*10 + 4;
          isPressed = true;
          break;
        case 4:
          tmp = tmp*10 + 5;
          isPressed = true;
          break;
        case 5:
          tmp = tmp*10 + 6;
          isPressed = true;
          break;
        case 6:
          tmp = tmp*10 + 7;
          isPressed = true;
          break;
        case 7:
          tmp = tmp*10 + 8;
          isPressed = true;
          break;
        case 8:
          tmp = tmp*10 + 9;
          isPressed = true;
          break;
        case 9:
          tmp = floor(tmp/10);
          isPressed = true;
          break;
        case 10:
          tmp = tmp*10;
          isPressed = true;
          break;
        case 11:
          return tmp;
          break;
      }
      if(isPressed) {
        if(tmp > 99999999) {
          tmp = floor((float)tmp / 10);
        }
        freshIntegerTextBox(tmp, textBoxBackground);
        delay(100);
        while(getButtonPressed(buttons, 12) != -1) {}
        delay(100);
        while(getButtonPressed(buttons, 12) != -1) {}
        isPressed = false;
      }
    }
  }
}
void drawIntegerScreen(Button* buttons, int size, uint16_t background, String comment, int integer, uint16_t textBoxBackground) {
  tft.fillScreen(background);
  drawButtons(buttons, size, background);
  tft.setTextSize(3);
  tft.setTextColor(getFrontColor(background));
  tft.setTextDatum(TL_DATUM);
  tft.drawString(comment, 19, 29);
  tft.drawSmoothRoundRect(9, 69, 10, 3, 300, 70, TFT_DARKCYAN, background);
  tft.fillSmoothRoundRect(12, 72, 295, 65, 8, textBoxBackground, TFT_DARKCYAN);
}
void freshIntegerTextBox(long integer, uint16_t background) {
  tft.setTextSize(4);
  tft.setTextDatum(BL_DATUM);
  tft.setTextColor(TFT_WHITE, background);
  tft.drawString(String(integer)+" ", 30, 120);
  tft.fillRect(30, 125, 270, 3, background);
}
void freshIntegerCursor(long integer, uint16_t background) {
  int digit = 0;
  uint8_t textWidth = tft.textWidth(" ");
  while(integer > 0) {
    integer = floor(integer / 10);
    ++digit;
  }
  if(millis()/500 % 2 == 0) {
    tft.fillRect(30 + digit*textWidth, 125, textWidth, 3, TFT_WHITE);
  } else {
    tft.fillRect(30 + digit*textWidth, 125, textWidth, 3, background);
  }
}

//Basic UI functions---------------------------
//To detect is the button pressed, -1 for none
int getButtonPressed(Button* buttons, int size) {
  int x = -1;
  int y = -1;
  touch.getTouchingPointNoBlocking(&x, &y);
  if(x == -1 || y == -1) {
    return -1;
  }
  for(int i = 0; i < size; ++i) {
    if(x > buttons[i].x && x < buttons[i].x+buttons[i].w && y > buttons[i].y && y < buttons[i].y+buttons[i].h) {
      return i;
    }
  }
  return -1;
}

//Simple buttons
void drawButtons(Button* buttons, int size, uint16_t background) {
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE);
  tft.setTextDatum(MC_DATUM);
  while(size > 0) {
    tft.fillSmoothRoundRect(buttons[size-1].x, buttons[size-1].y, buttons[size-1].w, buttons[size-1].h, 6, buttons[size-1].color, background);
    tft.drawString(buttons[size-1].name, 
                  buttons[size-1].x + (buttons[size-1].w / 2), 
                  buttons[size-1].y + (buttons[size-1].h / 2));
    --size;
  }
}

//parameters on top of main, setting and scan screen
void drawParam(uint16_t background) {
  uint16_t color = getFrontColor(background);

  tft.setTextSize(2);
  tft.setTextColor(color);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("Filename:", 9, 19);
  if(scan.filename == 0) {
    tft.drawString("<TIME>", 9, 39);
  } else {
    tft.drawString(String(scan.filename), 9, 39);
  }

  tft.drawString("Gain:", 169, 19);
  tft.setTextColor(THEME_ORG);
  tft.drawString(String(GainStr[scan.gain]), 169, 39);
  tft.setTextColor(color);
  tft.drawString("Time:", 169, 59);
  tft.setTextColor(THEME_ORG);
  tft.drawString(String(256-scan.integrationTime)+"c "+String((256-scan.integrationTime)*12/5)+"ms", 169, 79);
  tft.setTextColor(color);
  tft.drawString("Total time:", 169, 99);
  tft.setTextColor(TFT_SKYBLUE);
  int totalSeconds = ((long)scan.pixelsPerLine * scan.totalLines * motor[scan.pixelMotor].pulsePerPixel * motor[scan.pixelMotor].intervalTime 
                          + scan.totalLines * motor[scan.lineMotor].pulsePerPixel * motor[scan.lineMotor].intervalTime) / 1000000 * (2 - scan.isSShape);//calculate based on motor movement time
  tft.drawString(String((int)floor(totalSeconds/3600))+"h "+String((int)floor(totalSeconds/60%60))+"m "+String(totalSeconds%60)+"s", 169, 119);

  tft.setTextColor(TFT_SILVER);
  tft.setTextSize(1);
  tft.drawString("Pulse per pixel:"+String(motor[scan.pixelMotor].pulsePerPixel), 169, 139);
  tft.drawString("Interval time:"+String(motor[scan.pixelMotor].intervalTime)+"us", 169, 149);

  uint16_t x, y;
  getScanXY(&x, &y);
  float ratio = (float)70 / max(x,y);
  // Serial.println(ratio*x);
  // Serial.println(ratio*y);
  tft.drawRect(9, 89, ratio*x, ratio*y, color);
  drawArrowMatrix(9, 89, ratio*x, ratio*y, scan.path, color, 5);
  tft.setTextColor(color);
  tft.setTextSize(2);
  tft.setTextDatum(BC_DATUM);
  tft.drawString(String(x), 9+(ratio*x/2), 89-2);
  tft.setTextDatum(ML_DATUM);
  tft.drawString(String(y), 9+ratio*x+8, 89+(ratio*y/2));

  // tft.fillSmoothCircle((scan.startPoint == 0 || scan.startPoint == 3) ? 9 : 9 + ratio*x, scan.startPoint <= 1  ? 89 : 89 + ratio*y, 3, TFT_SKYBLUE, background);
  tft.fillSmoothCircle((scan.currentPoint == 0 || scan.currentPoint == 3) ? 9 : 9 + ratio*x, scan.currentPoint <= 1  ? 89 : 89 + ratio*y, 2, THEME_ORG, background);
  // tft.fillSmoothCircle(94, 59, 3, THEME_ORG, background);
  // tft.fillSmoothCircle(94, 74, 3, TFT_SKYBLUE, background);
  tft.setTextSize(1);
  tft.setTextColor(THEME_ORG);
  tft.setTextDatum(ML_DATUM);
  // tft.drawString("Current", 100, 60);
  // tft.drawString("Start", 100, 75);
  tft.drawString(SensorStr[scan.sensor], 94, 139);

  tft.setTextColor(color);
  if(scan.isSShape) {
    tft.setTextColor(THEME_GRN);
    tft.drawString("S Enable", 94, 149);
  } else {
    tft.setTextColor(TFT_SILVER);
    tft.drawString("S Disable", 94, 149);
  }
}

//Matrix to show on the parameter screen
void drawArrowMatrix(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t path, uint16_t color, int tipSize) {

  // path = 0;

  int lineArrowTipShift = 5;
  int lineArrowSideShift = 10;

  int pixelArrowTipShift = 20;
  int pixelArrowEndDistance = 10;
  int pixelArrowCount = max(floor((path < 4 ? h : w) / 15), 1);
  int pixelArrowStep = (path < 4 ? h : w) / pixelArrowCount - 3;
  int pixelArrowSideShift = ((path < 4 ? h : w) - (pixelArrowCount-1) * pixelArrowStep) / 2;

  switch(path) {
    case 0:
      drawArrow(x + w - lineArrowSideShift, y + lineArrowTipShift, h - 2*lineArrowTipShift, 1, color, tipSize);
      for(int i = 0; i < pixelArrowCount; ++i) {
        drawArrow(x + w - pixelArrowTipShift, y + pixelArrowSideShift + i*pixelArrowStep, w - pixelArrowTipShift - pixelArrowEndDistance, 2, color, tipSize);
      }
      break;
    case 1:
      drawArrow(x + lineArrowSideShift, y + lineArrowTipShift, h - 2*lineArrowTipShift, 1, color, tipSize);
      for(int i = 0; i < pixelArrowCount; ++i) {
        drawArrow(x + pixelArrowTipShift, y + pixelArrowSideShift + i*pixelArrowStep, w - pixelArrowTipShift - pixelArrowEndDistance, 3, color, tipSize);
      }
      break;
    case 2:
      drawArrow(x + w - lineArrowSideShift, y + h - lineArrowTipShift, h - 2*lineArrowTipShift, 0, color, tipSize);
      for(int i = 0; i < pixelArrowCount; ++i) {
        drawArrow(x + w - pixelArrowTipShift, y + pixelArrowSideShift + i*pixelArrowStep, w - pixelArrowTipShift - pixelArrowEndDistance, 2, color, tipSize);
      }
      break;
    case 3:
      drawArrow(x + lineArrowSideShift, y + h - lineArrowTipShift, h - 2*lineArrowTipShift, 0, color, tipSize);
      for(int i = 0; i < pixelArrowCount; ++i) {
        drawArrow(x + pixelArrowTipShift, y + pixelArrowSideShift + i*pixelArrowStep, w - pixelArrowTipShift - pixelArrowEndDistance, 3, color, tipSize);
      }
      break;
    case 4:
      drawArrow(x + w - lineArrowTipShift, y + lineArrowSideShift, w - 2*lineArrowTipShift, 2, color, tipSize);
      for(int i = 0; i < pixelArrowCount; ++i) {
        drawArrow(x + pixelArrowSideShift + i*pixelArrowStep, y + pixelArrowTipShift, h - pixelArrowTipShift - pixelArrowEndDistance, 1, color, tipSize);
      }
      break;
    case 5:
      drawArrow(x + w - lineArrowTipShift, y + h - lineArrowSideShift, w - 2*lineArrowTipShift, 2, color, tipSize);
      for(int i = 0; i < pixelArrowCount; ++i) {
        drawArrow(x + pixelArrowSideShift + i*pixelArrowStep, y + h - pixelArrowTipShift, h - pixelArrowTipShift - pixelArrowEndDistance, 0, color, tipSize);
      }
      break;
    case 6:
      drawArrow(x + lineArrowTipShift, y + lineArrowSideShift, w - 2*lineArrowTipShift, 3, color, tipSize);
      for(int i = 0; i < pixelArrowCount; ++i) {
        drawArrow(x + pixelArrowSideShift + i*pixelArrowStep, y + pixelArrowTipShift, h - pixelArrowTipShift - pixelArrowEndDistance, 1, color, tipSize);
      }
      break;
    case 7:
      drawArrow(x + lineArrowTipShift, y + h - lineArrowSideShift, w - 2*lineArrowTipShift, 3, color, tipSize);
      for(int i = 0; i < pixelArrowCount; ++i) {
        drawArrow(x + pixelArrowSideShift + i*pixelArrowStep, y + h - pixelArrowTipShift, h - pixelArrowTipShift - pixelArrowEndDistance, 0, color, tipSize);
      }
      break;
    default:
      Serial.println("Draw Arrow Matrix Path error!");
      break;
  }
}

//from 1 to 2, dir: 0:up, 1:down, 2:left, 3:right
void drawArrow(uint16_t x, uint16_t y, int16_t distance, uint8_t dir, uint16_t color, int tipSize) {
  // int tipSize = 5;//bigger for smaller tip
  int minTipSize = 3;
  int tipSharp = 7;
  //TODO:too small value will cause bug
  switch(dir) {
    case 0:
      tft.drawLine(x, y, x, y - distance, color);
      tft.drawLine(x, y - distance, x + max(round(distance/tipSize) - tipSharp,minTipSize), y - distance + max(round(distance/tipSize),minTipSize), color);
      tft.drawLine(x, y - distance, x - max(round(distance/tipSize) - tipSharp,minTipSize), y - distance + max(round(distance/tipSize),minTipSize), color);
      break;
    case 1:
      tft.drawLine(x, y, x, y + distance, color);
      tft.drawLine(x, y + distance, x + max(round(distance/tipSize) - tipSharp,minTipSize), y + distance - max(round(distance/tipSize),minTipSize), color);
      tft.drawLine(x, y + distance, x - max(round(distance/tipSize) - tipSharp,minTipSize), y + distance - max(round(distance/tipSize),minTipSize), color);
      break;
    case 2:
      tft.drawLine(x, y, x - distance, y, color);
      tft.drawLine(x - distance, y, x - distance + max(round(distance/tipSize),minTipSize), y + max(round(distance/tipSize) - tipSharp,minTipSize), color);
      tft.drawLine(x - distance, y, x - distance + max(round(distance/tipSize),minTipSize), y - max(round(distance/tipSize) - tipSharp,minTipSize), color);
      break;
    case 3:
      tft.drawLine(x, y, x + distance, y, color);
      tft.drawLine(x + distance, y, x + distance - max(round(distance/tipSize),minTipSize), y + max(round(distance/tipSize) - tipSharp,minTipSize), color);
      tft.drawLine(x + distance, y, x + distance - max(round(distance/tipSize),minTipSize), y - max(round(distance/tipSize) - tipSharp,minTipSize), color);
      break;
    default:
      Serial.println("Draw Arrow Path Error!");
      break;
  }
}

//Draw current time, used in param screen, isFull to print the whole time, refresh to faster
void drawTime(bool isFull) {
  now = rtc.now();

  if(isFull) {
    drawTimeFormat.textWidth = tft.textWidth(" ");
    tft.setTextSize(drawTimeFormat.size);
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(drawTimeFormat.color, drawTimeFormat.background);
    tft.drawString("    /  /     :  :", drawTimeFormat.x, drawTimeFormat.y);
  }

  if(prevTime.year() != now.year() || isFull) {
    tft.drawString(String(now.year()), drawTimeFormat.x, drawTimeFormat.y);
  }
  if(prevTime.month() != now.month() || isFull) {
    tft.drawString(String(now.month() < 10 ? "0" : "") + now.month(), drawTimeFormat.x + 5*drawTimeFormat.textWidth, drawTimeFormat.y);
  }
  if(prevTime.day() != now.day() || isFull) {
    tft.drawString(String(now.day() < 10 ? "0" : "") + now.day(), drawTimeFormat.x + 8*drawTimeFormat.textWidth, drawTimeFormat.y);
  }
  if(prevTime.hour() != now.hour() || isFull) {
    tft.drawString(String(now.hour() < 10 ? "0" : "") + now.hour(), drawTimeFormat.x + 11*drawTimeFormat.textWidth, drawTimeFormat.y);
  }
  if(prevTime.minute() != now.minute() || isFull) {
    tft.drawString(String(now.minute() < 10 ? "0" : "") + now.minute(), drawTimeFormat.x + 14*drawTimeFormat.textWidth, drawTimeFormat.y);
  }
  if(prevTime.second() != now.second() || isFull) {
    tft.drawString(String(now.second() < 10 ? "0" : "") + now.second(), drawTimeFormat.x + 17*drawTimeFormat.textWidth, drawTimeFormat.y);
  }

  // Serial.println(millis());
  prevTime = now;
}
void drawTime() {
  drawTime(true);
}
void refreshTime() {
  drawTime(false);
}

void setDrawTimeFormat(uint16_t x, uint16_t y, uint16_t color, uint16_t background, uint8_t size) {
  drawTimeFormat = {.x = x, .y = y, .color = color, .background = background, .size = size};
}

uint16_t getFrontColor(uint16_t background) {
  // uint16_t colorSum = (background<<3 & B11111000) + (background>>3 & B11111100) + (background>>8 & B11111000);
  // Serial.println("Background:"+String(background)+", Sum:"+String(colorSum));
  // Serial.println((background<<3 & B11111000));
  // Serial.println((background>>3 & B11111100));
  // Serial.println((background>>8 & B11111000));
  //if the background is light, front will be dark
  return ((background<<3 & B11111000) + (background>>3 & B11111100) + (background>>8 & B11111000)) > 450 ? TFT_BLACK : TFT_WHITE;
}

//Mainly functions---------------------------------
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

  //Pixel direction select
  switch (scan.path) {
    case 0:
    case 2:
    case 4:
    case 6:
      scan.pixelDirection = MOTOR_FORWARD;
      break;
    case 1:
    case 3:
    case 5:
    case 7:
      scan.pixelDirection = MOTOR_BACKWARD;
      break;
  }

  //Line direction select
  switch (scan.path) {
    case 0:
    case 1:
    case 4:
    case 5:
      scan.lineDirection = MOTOR_FORWARD;
      break;
    case 2:
    case 3:
    case 6:
    case 7:
      scan.lineDirection = MOTOR_BACKWARD;
      break;
  }
  
}

//TODO:Add more sensors
void scanTask() {
  tft.fillScreen(TFT_BLACK);
  String dir;

  if(scan.filename == 0) {
    now = rtc.now();
    dir = String(now.year()) + "-" + String(now.month()) + "-" + String(now.day()) + " " + String(now.hour()) + "-" + String(now.minute()) + "-" + String(now.second());
  } else {
    dir = String(scan.filename);
  }

  lcd.setCursor(0, 0);
  lcd.print("Name:");
  lcd.print(dir);
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

  String bmpDir = dir;
  bmpDir.concat(".bmp");
  bmp = SD.open(bmpDir, FILE_WRITE);
  for (int i = 0; i < 54; i++) {
    bmp.write(bmpHead[i]);
  }

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
  Serial.println(String(256-scan.integrationTime)+" cycles, "+String((256-scan.integrationTime)*12/5)+"ms.");
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

void scanPreview() {
  Serial.println(scan.pixelDirection);
  Serial.println(scan.lineDirection);
  enableTimer();
  setMove(scan.pixelMotor, scan.pixelDirection, 0);
  movePixel(scan.pixelsPerLine);
  setMove(scan.lineMotor, scan.lineDirection, 0);
  movePixel(scan.totalLines);
  setMove(scan.pixelMotor, !scan.pixelDirection, 0);
  movePixel(scan.pixelsPerLine);
  setMove(scan.lineMotor, !scan.lineDirection, 0);
  movePixel(scan.totalLines);
  disableTimer();
}

void moveTask() {
  Serial.println(scan.pixelDirection);
  Serial.println(scan.lineDirection);
  enableTimer();
  setMove(scan.pixelMotor, scan.pixelDirection, 0);
  movePixel(scan.pixelsPerLine);
  setMove(scan.lineMotor, scan.lineDirection, 0);
  movePixel(scan.totalLines);
  disableTimer();
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

void computerControl() {
  scan.filename = getPCInt("Last filename is: " + String(scan.filename) + ". \nEnter filename(0 for movement):") % 100000000;//TODO:read filename from sd card
  Serial.println(scan.filename);
  scan.path = getPCInt("Enter path(0-7): ");
  Serial.println(pathComment[scan.path]);
  scan.pixelsPerLine = getPCInt("Enter pixelsPerLine:");
  Serial.println(scan.pixelsPerLine);
  scan.totalLines = getPCInt("Enter totalLines:");
  Serial.println(scan.totalLines);

  if(scan.filename == 0) {
    Serial.println("Move task");
    setScanMotor();
    moveTask();
  } else {
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
}

void getScanXY(uint16_t* x, uint16_t* y) {
  if(scan.path >= 0 && scan.path <= 3) {
    *x = scan.pixelsPerLine;
    *y = scan.totalLines;
  } else {
    *x = scan.totalLines;
    *y = scan.pixelsPerLine;
  }
}

void setScanXY(uint16_t x, uint16_t y) {
  if(scan.path >= 0 && scan.path <= 3) {
    scan.pixelsPerLine = x;
    scan.totalLines = y;
  } else {
    scan.totalLines = x;
    scan.pixelsPerLine = y;
  }
}

//Sensor----------------------------
void readColor(uint16_t* red, uint16_t* green, uint16_t* blue) {
  *red = tcs.read16(TCS34725_RDATAL);
  *green = tcs.read16(TCS34725_GDATAL);
  *blue = tcs.read16(TCS34725_BDATAL);
}

//Motor------------------------------
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

//this function is deprecated
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

//Data-------------------------------
//Deprecated
// void saveData(uint64_t addr, uint8_t red, uint8_t green, uint8_t blue) {
//   vti.write(3 * addr, red);
//   vti.write(3 * addr + 1, green);
//   vti.write(3 * addr + 2, blue);
// }

//Little-endian ordering
void SDWrite16(File file, uint16_t data) {
  file.write(data % 256);//TODO:try | 0xFF
  file.write(floor(data / 256));//TODO:try >>8
}

//Interaction---------------------
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

//Display---------------------------
void printColor(int red, int green, int blue) {
  Serial.print("R"); Serial.print(red, DEC); Serial.print(" ");
  Serial.print("G"); Serial.print(green, DEC); Serial.print(" ");
  Serial.print("B"); Serial.print(blue, DEC); Serial.println(" ");
}

