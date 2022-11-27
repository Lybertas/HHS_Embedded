#include <Adafruit_Microbit.h>;

Adafruit_Microbit_Matrix microbit;


void setup() {
  // just for logging
  Serial.begin(9600);
  Serial.println("Begin!");

  microbit.begin();
}


void leftToRight(int row) {
  for (int i = 0; i < 5; i++) {
    microbit.drawPixel(i, row, LED_ON);
    delay(150);
    microbit.drawPixel(i, row, LED_OFF);
  }
}

void rightToLeft(int row) {
  for (int i = 4; i >= 0; i--) {
    microbit.drawPixel(i, row, LED_ON);
    delay(150);
    microbit.drawPixel(i, row, LED_OFF);
  }
}

void loop() {
  
  leftToRight(1);
  rightToLeft(2);
  leftToRight(3);
  rightToLeft(2);
}