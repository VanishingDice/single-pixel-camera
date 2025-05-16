/*

Jimmy Zhang
2025.5.14
V0.1.0
*/

#include <SPI.h>

#include <TFT_eSPI.h> // Hardware-specific library

//For screen touching pin
#define XM A0 //X-
#define YM A1 //Y-
#define XP A2 //X+
#define YP A3 //Y+
#define SCREEN_ROTATION 3
#define SCREEN_XPIXELS 480
#define SCREEN_YPIXELS 320
int xShift = -50;
int yShift = -50;
float xRatio = 1.23;
float yRatio = 1.29;
uint8_t filterCycles = 5;//How many point to be average
uint8_t filterMaxAllowableDrift = 50;//limit length between two pixels

TFT_eSPI tft = TFT_eSPI();       // Invoke custom library

void setup(void) {
  Serial.begin(115200);

  tft.init();
  tft.setRotation(SCREEN_ROTATION);

  tft.fillScreen(tft.color565(0, 0, 0));

  // tft.fillRect(100, 100, 100, 50, tft.color565(200, 100, 100));

  // tft.fillRect(300, 100, 100, 50, tft.color565(100, 200, 100));

}

void loop() {
  uint16_t x;
  uint16_t y;

  // getTouchingPoint(&x, &y);

  // if(x > 100 && x < 200 && y > 100 && y < 150) {
  //   tft.fillRect(100, 100, 100, 50, tft.color565(150, 100, 100));
  //   delay(300);
  //   tft.fillRect(100, 100, 100, 50, tft.color565(200, 100, 100));
  // }
  // if(x > 300 && x < 400 && y > 100 && y < 150) {
  //   tft.fillRect(300, 100, 100, 50, tft.color565(100, 150, 100));
  //   delay(300);
  //   tft.fillRect(300, 100, 100, 50, tft.color565(100, 200, 100));
  // }

  drawTouchingPoint();

  // delay(100);

  // wait();

}

void wait() {
  while(digitalRead(10) == HIGH) {}
  delay(100);
}

void drawTouchingPoint() {
  // tft.fillScreen(tft.color565(0, 0, 0));
  int16_t x = -1;
  int16_t y = -1;
  getRawTouchingPoint(&x, &y);
  // Serial.println("X:"+String(x)+", Y:"+String(y));
  tft.drawPixel(x, y, tft.color565(0, 255, 0)); 
  // delay(1000);
}

void getTouchingPoint(uint16_t* x, uint16_t* y) {
  int16_t xTmp[filterCycles];
  int16_t yTmp[filterCycles];
  for(int i = 0; i < filterCycles; ++i) {
    while(1) {
      getRawTouchingPoint(&xTmp[i], &yTmp[i]);
      if((xTmp[i] == -1) || (yTmp[i] == -1)) { 
        i = 0;
        continue;//when the finger left the screen, restart count
      }
      if(i == 0) {
        break;//if it's the first data
      } else if((abs(xTmp[i-1]-xTmp[i]) + abs(yTmp[i-1]-yTmp[i])) <= filterMaxAllowableDrift) {
        break;//if coming data is in the filter limit
      }
    }
  }

  *x = 0;
  *y = 0;

  for(int i = 0; i < filterCycles; ++i) {
    *x += xTmp[i];
    *y += yTmp[i];
  }

  *x /= filterCycles;
  *y /= filterCycles;

}

void getRawTouchingPoint(int16_t* x, int16_t* y) {
  int16_t xRaw = -1;
  int16_t yRaw = -1;
  pinMode(YP, OUTPUT);
  pinMode(YM, OUTPUT);
  pinMode(XP, INPUT_PULLUP);
  pinMode(XM, INPUT_PULLUP);
  digitalWrite(YP, HIGH);
  digitalWrite(YM, LOW);
  int16_t tmp;
  analogRead(XP);//The first time to do the analogRead will cause a wrong result, don't know why, it should be 101x, but it's between 940 and 1000 at first time
  tmp = analogRead(XP);
  // Serial.print("xTmp+:"+String(tmp));
  // Serial.print(", xTmp-:"+String(analogRead(XM)));
  if(tmp > 1000) {
    xRaw = -1;
  } else {
    xRaw = ((long)tmp*SCREEN_XPIXELS/1024+xShift)*xRatio;
  }

  pinMode(XP, OUTPUT);
  pinMode(XM, OUTPUT);
  pinMode(YP, INPUT_PULLUP);
  pinMode(YM, INPUT_PULLUP);
  digitalWrite(XP, HIGH);
  digitalWrite(XM, LOW);
  analogRead(YP);//Same to this
  tmp = analogRead(YP);
  // Serial.print(", yTmp+:"+String(tmp));
  // Serial.println(", yTmp-:"+String(analogRead(YM)));
  if(tmp > 1000) {
    yRaw = -1;
  } else {
    yRaw = ((long)tmp*SCREEN_YPIXELS/1024+yShift)*yRatio;
  }

  rawToScreenCoordinate(xRaw, yRaw, x, y);
}

void rawToScreenCoordinate(int16_t xRaw, int16_t yRaw, int16_t* x, int16_t* y) {
  switch(SCREEN_ROTATION) {
    case 0:
      *x = yRaw;
      *y = xRaw;
      break;
    case 1:
      *x = xRaw;
      *y = SCREEN_YPIXELS - yRaw;
      break;
    case 2:
      *x = SCREEN_YPIXELS - yRaw;
      *y = SCREEN_XPIXELS - xRaw;
      break;
    case 3:
      *x = SCREEN_XPIXELS - xRaw;
      *y = yRaw;
      break;
  }

}



