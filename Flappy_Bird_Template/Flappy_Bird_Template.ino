// Flappy Bird game, deel van het practicum 'Technical Skills 2: Embedded Systems'
// Gert den Neijsel, Networks & Systems Engineering, HBO-ICT, Haagse Hogeschool, 2022
// De game doet het nu nog niet. Er ontbreken een aantal belangrijke onderdelen. Volg de practicumhandleiding om deze in te vullen.
// Tip: Doe aan versiebeheer. Sla elke tussenversie met een eigen timestamp op (dat doet de Arduino IDE vanzelf al bij 'opslaan als').

#include <LSM303AGR_ACC_Sensor.h>  // t.b.v. bewegingssensor
#include <Adafruit_Microbit.h>     // t.b.v. aansturing display

#define DEV_I2C Wire1  // Wire1 is voor de interne I2C bus van de Microbit V2
// #define DEV_I2C Wire  // Als je een Microbit V1 hebt, Comment dan de regel hierboven en Uncomment deze.
#define SPEAKER 27  // pin waarop de luidspreker zit bij de Microbit V2, de V1 heeft geen luidspreker

// Vanaf hier tot setup() zijn allemaal globale variabelen die in elk deel van het programma en in elke functie bruikbaar zijn:
LSM303AGR_ACC_Sensor Acc(&DEV_I2C);  // t.b.v. bewegingssensor
Adafruit_Microbit_Matrix display;    // t.b.v. aansturing display

uint8_t matrix[5];                         // uint8_t betekent: een unsigned 8 bits integer
unsigned long tijdstipNu = 0;              // tijdstip
unsigned long tijdstipMuurVerplaatst = 0;  // tijdstip
unsigned long tijdstipKnopGedrukt = 0;     // tijdstip
unsigned long muurInterval = 1000;         // tijdsduur 1sec
int gat = 0;                               // plek waar het gat in de muur zit.
int bird = 2;                              // plek waar de vogel zit. In het midden beginnen.
int vorigeBird = 0;
int afstandTussenMuren = 0;  // teller
int score = 0;               // teller
int toonhoogte = 600;        // beginwaarde voor 'beep', hogere waarde is lagere toon
bool flap = 1;               // t.b.v. het knipperen van de vogel
bool muurIsVerschoven = 0;   // op de moment dat de muur verschoven is doen we onze controles

int lastStateA = 0; // eigen waardes om knoppen 'beter' te laten werken
int lastStateB = 0;

// Forward declarations van functies. Dit voorkomt dat functies niet gevonden kunnen worden.
// Bij het compileren van het programma wordt het bestand regel voor regel gelezen totdat de setup() en loop() bereikt zijn.
// Als daarin functies worden aangeroepen die nog niet gezien zijn, krijg je foutmeldingen van de compiler.
// De eigenlijke functies staan onderaan.
void controleerScore();
void verplaatsMuur();
void youWin();
void gameOver();
void startOver();
void beep();
void startSound();
void endSound();
void bepaalPositieMetKnoppen(int&);
void bepaalPositieMetAccelerometer(int&);

void setup() {
  DEV_I2C.begin();            // i2c aanzetten
  Acc.begin();                // accelerometer aanzetten
  Acc.Enable();               // accelerometer starten
  randomSeed(analogRead(2));  // pin 2 is niet aangesloten en genereert ruis; basis voor 'random'
  Serial.begin(9600);         // seriële poort starten met een snelheid van 9600 bits per seconde
  Serial.println("Welkom bij Flappy Bird!");
  display.begin();  // matrix display aanzetten
  // Stap 8. Plaats hieronder de commando's waarmee je de pinnen van beide schakelaars initialiseert.
  // Je mag hiervoor de labels PIN_BUTTON_A en PIN_BUTTON_B gebruiken. Dit zijn constanten uit de Adafruit_Microbit.h library.
  pinMode(PIN_BUTTON_A, INPUT);
  pinMode(PIN_BUTTON_B, INPUT);
  // Ja, hierboven dus...
  startSound();  // geluidje maken bij het opstarten
}

void loop() {
  // bron van onderstaande regel m.b.t. tijdafhandeling, zie: Voorbeelden->02.Digital->BlinkWithoutDelay
  tijdstipNu = millis();
  controleerScore();
  verplaatsMuur();
}

void controleerScore() {
  // bron van onderstaande regels m.b.t. knopafhandeling, zie: Voorbeelden->02.Digital->StateChangeDetection
  if (tijdstipNu - tijdstipKnopGedrukt >= 50) {  // deze lus wordt elke 50 milliseconde doorlopen om de score te checken, is tevens de knippersnelheid

    bepaalPositieMetKnoppen(bird);  // dit levert een waarde van 0 .. 4 op (de positie van de vogel)
    bepaalPositieMetAccelerometer(bird); // dit levert een waarde van 0 .. 4 op (de positie van de vogel)

    int aantalLedsAan = 0;
    if (muurIsVerschoven == 1) {
      int ledUit = 5;  // met opzet buiten het bereik van 0..4 gekozen
      for (int y = 0; y < 5; y++) {
        if (matrix[y] & 0b10000) {  // in de meest linker kolom de leds tellen die aan staan
          aantalLedsAan++;          // als het er 4 zijn, dan is het een muur
        } else {
          ledUit = y;  // als er straks 4 leds aan blijken te zijn, dan was y de locatie van het gat.
        }
      }
      if (aantalLedsAan == 4) {
        if (ledUit == bird) {  // vogel door opening
          score++;             // score wordt met 1 opgehoogd
          beep();
          Serial.print("Score: ");
          Serial.println(score);
        } else {
          gameOver();
        }
      }
      muurIsVerschoven = 0;
    }

    if (flap == 1) {  // met het knipperen simuleren we het flapperen
      // Stap 5. Vul de regels hieronder aan met een bitwise operation zodat de meest linker led wordt aangezet.
      matrix[bird] = matrix[bird]+16;  // vogel zit in de meest linker kolom
      flap = 0;
    } else {
      // Stap 6. Vul de regels hieronder aan met een bitwise operation zodat de meest linker led wordt uitgezet.
      // Het is in beide gevallen dezelfde bitwise operation.
      matrix[vorigeBird] = matrix[vorigeBird]-0b10000;  // oude sporen uitwissen (de meest linker led uitzetten)
      matrix[bird] = matrix[bird]-0b10000;              // de meest linker led uitzetten
      flap = 1;
    }

    display.show(matrix);  // scherm updates tonen
    vorigeBird = bird;
    tijdstipKnopGedrukt = tijdstipNu;
  }
}

void verplaatsMuur() {
  if (tijdstipNu - tijdstipMuurVerplaatst >= muurInterval) {  // het interval waarmee de muur opschuift
    tijdstipMuurVerplaatst = tijdstipNu;

    for (int y = 0; y < 5; y++) {
      // Stap 1
      // Vul de regel hieronder aan met een bitwise operation zodat alle rijen één hele kolom naar links opschuiven.
      matrix[y] = matrix[y]*2;           // vul aan vóór de ;
      matrix[y] = matrix[y] & 0b11111;  // beperk het getal tot 5 bits en gooi alles daarboven weg
    }

    afstandTussenMuren++;
    if (afstandTussenMuren > 2) {  // muren staan 2 rijen van elkaar (2 open rijen ertussen)

      int vorigeGat = gat;
      while (gat == vorigeGat) {  // om te zorgen dat je altijd een gat op een nieuwe locatie hebt.
        gat = random(5);          // de led die uit is in de muur, willekeurig getal 0 .. 4
      }

      for (int y = 0; y < 5; y++) {
        // Stap 2
        // Vul de regel hieronder aan met een bitwise operation zodat alle rijen aan de rechterkant met een 1 gevuld worden.
        matrix[y] |= matrix[y]+1;  // meest rechter kolom vullen met 1-en
        if (gat == y) {
          // Stap 3
          // Vul de regel hieronder aan met een bitwise operation zodat in alle rijen het betreffende bitje wordt uitgezet.
          matrix[y] = matrix[y]^1;  // ledje y uitzetten (het gat)
        }
      }
      afstandTussenMuren = 0;
    }
    muurIsVerschoven = 1;  // om aan te geven dat er iets veranderd is.

    muurInterval = muurInterval - 10;   // telkens wat sneller en moelijker maken
    toonhoogte = toonhoogte - 2;        // we voeren de 'spanning' op
    if (muurInterval <= 100) youWin();  // als je op een interval van 100ms uitkomt, dan ben je wel heel goed (=snel)
  }
}

void youWin() {
  display.show(display.YES);
  Serial.print("You win!");
  startOver();
}

void gameOver() {
  display.show(display.NO);  // draw a NO cross
  Serial.println("You lost!");
  endSound();
  display.print("Score:");
  display.print(score);
  startOver();
}

void startOver() {
  Serial.print("Final Score: ");
  Serial.println(score);
  Serial.print("Final Game speed (lower is faster, 100 is fastest): ");
  Serial.println(muurInterval);
  while (digitalRead(PIN_BUTTON_A) && digitalRead(PIN_BUTTON_B))  // beide knoppen tegelijk indrukken om spel opnieuw te starten
    ;                                                             // zolang knop niet ingedrukt wordt
  Serial.println("New game!");
  // Spel opnieuw initialiseren, scherm leegmaken
  for (int y = 0; y < 5; y++) {
    matrix[y] = 0b00000;
  }
  bird = 2;  // middenpositie
  score = 0;
  muurInterval = 1000;
  delay(1000);
  startSound();
}

void beep() {
  pinMode(SPEAKER, OUTPUT);
  for (int n = 0; n < 30; n++) {
    digitalWrite(SPEAKER, HIGH);    // turn the speaker on (HIGH is the voltage level)
    delayMicroseconds(toonhoogte);  // wait for a second
    digitalWrite(SPEAKER, LOW);     // turn the speaker off by making the voltage LOW
    delayMicroseconds(toonhoogte);  // wait for a second
  }
}

void startSound() {
  // Geluidje maken, effect is experimenteel bepaald
  pinMode(SPEAKER, OUTPUT);
  for (int n = 3000; n > 100; n = n - 10) {
    digitalWrite(SPEAKER, HIGH);
    delayMicroseconds(n);
    digitalWrite(SPEAKER, LOW);
    delayMicroseconds(n);
  }
}

void endSound() {
  // Geluidje maken, effect is experimenteel bepaald. Leek wel game-achtig
  pinMode(SPEAKER, OUTPUT);
  for (int m = 0; m < 3; m++) {
    for (int n = 100; n < 2500; n = n + 15) {
      digitalWrite(SPEAKER, HIGH);
      delayMicroseconds(n);
      digitalWrite(SPEAKER, LOW);
      delayMicroseconds(n);
    }
  }
}

void bepaalPositieMetKnoppen(int& vogel) {

  // Je moet in de setup() nog iets toevoegen zodat dit werkt (Stap 7). Heb je dat al gedaan?
  int buttonAstate = !digitalRead(PIN_BUTTON_A);
  int buttonBstate = !digitalRead(PIN_BUTTON_B);
  
  if (buttonAstate != lastStateA){
    if(buttonAstate == HIGH && vogel > 0){
      vogel--;
    }
  }
  if (buttonBstate != lastStateB) {
    if (buttonBstate == HIGH && vogel < 4) {
      vogel++;
    }
  } 
  lastStateA = buttonAstate;
  lastStateB = buttonBstate;
}

void bepaalPositieMetAccelerometer(int& vogel) {
  // Read accelerometer LSM303AGR.
  int32_t accelerometer[3];
  Acc.GetAxes(accelerometer);
  vogel = map(accelerometer[1],500,-500,4,0);  // hier gebeurt alle magie. Wijzig deze regel en kijk wat verder nodig is.
}