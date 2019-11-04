#include <Servo.h>
#include <IRremote.h>
#define linePinL A5
#define linePinR A4
#define linePinC A3
#define RECEIVERS 3
#define irRFrontPin 7
#define irRLeftPin 6
#define irRRightPin 5
#define sonarPing 2
#define red 8
#define green 9
#define blue 10
IRrecv *irrecvs[RECEIVERS];
decode_results results;
IRsend irsend;
Servo right;
Servo left;
/*
   *****Progression*****
   Line Following: COMPLETE --- pidSteer()
   IR transmition: sending write code --- sendHit()
   IR Receving/LED: COMPLETE --- irRecv()
   Backup control method: in progress --- turnAt()
   remote control: none
*/
int lastHit, cTime, lineL, lineC, lineR, tp, error, offset, lastError, intergral, derivative;
float kp, ki, kd, steer;
bool hit;
void setup() {
  hit = false;
  lastError = 0;
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(8), irRecv(), CHANGE);
  Serial.println("init");
  irrecvs[0] = new IRrecv(irRLeftPin);
  irrecvs[1] = new IRrecv(irRFrontPin);
  irrecvs[2] = new IRrecv(irRRightPin);

  for (int i = 0; i < RECEIVERS; i++) {
    irrecvs[i]->enableIRIn();
  }

  offset = 700;
  kp = 1.36;
  ki = 0.000001;
  kd = 0.0000001;

  pinMode(linePinL, INPUT_PULLUP);
  pinMode(linePinR, INPUT_PULLUP);
  pinMode(linePinC, INPUT_PULLUP);
  pinMode(sonarPing, OUTPUT);
  pinMode(red, OUTPUT);
  pinMode(green, OUTPUT);
  pinMode(blue, OUTPUT);
  left.attach(13);
  right.attach(12);
  rgb(0, 0, 0);
  rgb(0, 255, 0);
  delay(2000);
  rgb(0, 0, 0);
}

float pidSteer() {
  lineL = analogRead(linePinL);
  lineC = analogRead(linePinC);
  Serial.println(String(lineL) + "   " + String(lineC) + "   " + String(lineR));
  error =  offset - lineC;
  intergral += error;
  derivative = error - lastError;
  steer = (kp * error) + (ki * intergral) + (kd * derivative);
  error = lastError;
  drive(steer);
  return steer;
}

void drive(float ofs) {
  left.write(350 - ofs);
  right.write(10 + ofs);
}



void collisionDetection() {
  if (readSonar(sonarPing) < 32) {
    left.detach();
    right.detach();
    irRecv();
    left.attach(13);
    right.attach(12);
  }
}

long readSonar(int pingPin) {
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);
  pinMode(pingPin, INPUT);
  long duration = pulseIn(sonarPing, HIGH);
  long cm = duration / 29 / 2;
  Serial.println(cm);
  return cm;
}

int turnAt() {
  //Serial.println(readSonar(sonarPing));
  if (readSonar(sonarPing) < 15) {
    drive(75);
  }
  else {
    drive(0);
  }
}
void sendHit() {
  for (int i  = 0; i < 3; i++) {
    irsend.sendSony(0x5A5 , 3);
    //Serial.println("send hit " + String(i));
    delay(200);
  }
}
void rgb(uint8_t r, uint8_t g, uint8_t b) {
  digitalWrite(red, r);
  digitalWrite(green, g);
  digitalWrite(blue, b);
}
void element(uint8_t r, uint8_t g, uint8_t b) {
  rgb(r, g, b);
  cTime = millis();
  hit = false;
}

int irRecv() {
  if (hit) {
    if (millis() - lastHit > 5000) {
      rgb(0, 0, 0);
      hit = false;
    }
    for (int i = 0; i < RECEIVERS; i++) {
      irrecvs[i]->enableIRIn();
      delay(40);
    }
    for (int i = 0; i < 2; i++) {
      //    Serial.println ("get results");
      if (irrecvs[0]->decode(&results))    {
        //      Serial.println("decode");
        Serial.println(results.value, HEX);
        if (results.value == 0xB13) {
          Serial.println("water");
          element(0, 64, 128);
        } else if (results.value == 0xC9A) {
          Serial.println("grass");
          element(0, 255, 0);
        } else if (results.value == 0xEA9) {
          Serial.println("earth");
          element(128, 20, 128);
        } else if (results.value == 0xA19) {
          Serial.println("air");
          element(255, 255, 255);
        } else if (results.value == 0xE1E) {
          Serial.println("electricty");
          element(128, 128, 0);
        } else if (results.value == 0xF19) {
          Serial.println("fire");
          element(255, 55, 0);
        } else if (results.value == 0x5A5) {
          Serial.println("hit");
          if (millis() - lastHit > 5000) {
            rgb(128, 0, 0);
            lastHit = millis();
            hit = true;
          }
          else {
            //rgb(0, 0, 0);
          }
        }
      }
      irrecvs[i]->resume();
    }
  }
}

void loop() {

  //pidSteer();
  //turnAt();
  irRecv();
  //sendHit();
  //  if (millis() - cTime > 5000) {
  //    rgb(0, 0, 0);
  //  }
}
