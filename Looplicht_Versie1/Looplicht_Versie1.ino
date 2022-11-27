#include <Adafruit_Microbit.h>;

Adafruit_Microbit_Matrix microbit;
const int TIMER = 150; // hoger getal == slomer, lager getal == sneller
const int COL[] = {4,7,3,6,10};

void setup() {
  // just for logging
  Serial.begin(9600);
  Serial.println("Begin!");
  pinMode(23, OUTPUT);
}


void leftToRight() {
  for (int i = 0; i < 5; i++) {
    pinMode(COL[i], OUTPUT);
    analogWrite(23, 255);
    delay(TIMER);
    pinMode(COL[i], INPUT);
    delay(TIMER);
  }
}

void rightToLeft() {
  for (int i = 4; i >= 0; i--) {
    pinMode(COL[i], OUTPUT);
    analogWrite(23, 255);;
    delay(TIMER);
    pinMode(COL[i], INPUT);
    delay(TIMER);
  }
}

void loop() {
  leftToRight();
  rightToLeft();
}