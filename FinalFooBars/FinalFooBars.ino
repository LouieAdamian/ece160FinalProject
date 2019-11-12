#include <Servo.h>
#include <PS2X_lib.h>
#include <IRremote.h>
//line folowing
#define linePinL A5
#define linePinR A4
#define linePinC A3
//ir recevers
#define RECEIVERS 3
#define irRFrontPin 7
#define irRLeftPin 6
#define irRRightPin 5
#define sonarPing 2
// ir codes
#define fireCode 0xF19
#define waterCode 0xB13
#define grassCode 0xC9A
#define earthCode 0xEA9
#define airCode 0xA19
#define elecCode 0xE1E
#define hitCode 0x5A5
//colors
#define red 8 //t
#define green 9 //t
#define blue 10 //t
//ps2x
#define PS2_DAT A0
#define PS2_CMD A1
#define PS2_SEL A2
#define PS2_CLK 4
#define pressures   false
#define rumble      false
const uint8_t colors[8][3] = {
  {255, 55, 0},//orange 0 ch
  {1, 255, 0},//green 1 ch
  {128, 20, 255},//brown 2 ch
  {255, 255, 255},//white 3 ch
  {128, 255, 0},//yellow 4
  {0, 64, 255},//blue 5 ch
  {128, 0, 0},//red ch
  {0, 0, 0}
};
static uint8_t pins[] = {5, 6, 7};

IRrecv irrecvs[RECEIVERS] = {IRrecv(irRFrontPin), IRrecv(irRLeftPin), IRrecv(irRRightPin)};
decode_results results;
IRsend irsend;
Servo right;
Servo left;
PS2X ps2x;

/*
   *****Progression*****
   Line Following: COMPLETE --- pidSteer()
   IR transmition: sending write code --- sendHit()
   IR Receving/LED: COMPLETE --- irRecv()
   remote control: PS2 -- readPS()
   Backup control method: in progress --- turnAt()
*/

int lastHit, cTime, ledT, lineL, lineC, lineR, tp, error, offset, lastError, intergral, derivative, codeReceived;
float kp, ki, kd, steer;
int pserr = 0;
byte type = 0;
byte vibrate = 0;

void setup() {
  lastError = 0;
  Serial.begin(9600);
  Serial.println("init");
  for (int i = 0; i < RECEIVERS; i++) {
    irrecvs[i].enableIRIn();
  }
  pserr = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
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
  //blinkC(7);
  //autoMode();
  for (int i = 0; i > 7; i++) {
    analogWrite(red, colors[i][0]);
    analogWrite(green, colors[i][1]);
    analogWrite(blue, colors[i][2]);
    delay(1000);
    Serial.println(i);
  }
  autoMode();
}
void blinkC(uint8_t index) {
  analogWrite(red, colors[index][0]);
  analogWrite(green, colors[index][1]);
  analogWrite(blue, colors[index][2]);
  ledT = millis();
}
float pidSteer() {
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

void readPS() {
  ps2x.read_gamepad(false, vibrate);
  if ( ps2x.Button(PSB_TRIANGLE)) {
    drive(0);
    Serial.println("triangle");
  }
  else if (ps2x.Button(PSB_CROSS)) {
    drive(360);
    Serial.println("cross");

  }
  else if (ps2x.Button(PSB_CIRCLE)) {
    Serial.println("circle");

    drive(180);
  }
  else if (ps2x.Button(PSB_SQUARE)) {
    drive(-180);
  }
  if(ps2x.Button(PSB_PAD_DOWN)){
    sendHit();
  }
  else {
    left.write(180);
    right.write(180);
  }
}
void sendHit() {
  for (int i  = 0; i < 3; i++) {
    irsend.sendSony(0x5A5 , 3);
    Serial.println("send hit " + String(i));
    delay(200);
  }
}

int irRecv() {
  int codeReceived = 0;
  if (ledT) {
    if (ledT - millis() > 5000) {                               // Reset Ton to zero in order to receive
      for (int i = 0; i < RECEIVERS; i++) {          // ReEnable the receivers when the LED goes off. Put here so that don't get buffer overflow in receiver.
        irrecvs[i].enableIRIn();
      }
    }
  }
  else {
    for (int i = 0; i < RECEIVERS; i++) {
      irrecvs[i] = IRrecv(pins[i]);
      irrecvs[i].enableIRIn();
      delay(100);
      if (irrecvs[i].decode(&results)) {
        Serial.println("check 3");
        if ((results.value == waterCode) || (results.value == airCode) ||
            (results.value == grassCode) || (results.value == fireCode) ||
            (results.value == hitCode) || (results.value == earthCode) ||
            (results.value == elecCode)) {
          codeReceived = results.value;
        }
      }
      Serial.println(results.value, HEX);
    }
  }
  switch (codeReceived) {
    case fireCode:
      blinkC(0);
      Serial.println("fire");
      break;

    case waterCode:
      blinkC(5);
      Serial.println("water");
      break;

    case earthCode:
      blinkC(2);
      Serial.println("earth");
      break;

    case airCode:
      blinkC(3);
      Serial.print("air");
      break;

    case elecCode:
      blinkC(4);
      Serial.println("elec");
      break;

    case grassCode:
      blinkC(1);
      Serial.println("grass");
      break;

    case hitCode:
      blinkC(6);
      Serial.println("hit");
      break;

    default:  // default is to leave the RGB LED in last state
      break;
  }
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
int autoMode() {
  while (millis() < 40000) {
    pidSteer();
    irRecv();
  }
}

void loop() {
    readPS();
  //pidSteer();
  //turnAt();
  irRecv();
 // sendHit();
}
