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
#define red 8
#define green 9
#define blue 10
#define orangeStart 0
#define greenStart 3
#define brownStart 6
#define whiteStart 9
#define yellowStart 12
#define blueStart 15
#define redStart 18
//ps2x
#define PS2_DAT A0
#define PS2_CMD A1
#define PS2_SEL A2
#define PS2_CLK 4
#define pressures   false
#define rumble      false
const uint8_t colors[8][3] = {{255, 55, 0},
  {0, 255, 0},
  {128, 20, 128},
  {255, 255, 255},
  {128, 128, 0},
  {0, 64, 128},
  {128, 0, 0},
  {0, 0, 0}
};
IRrecv *irrecvs[RECEIVERS];
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
  attachInterrupt(digitalPinToInterrupt(8), irRecv(), CHANGE);
  Serial.println("init");
  irrecvs[0] = new IRrecv(irRLeftPin);
  irrecvs[1] = new IRrecv(irRFrontPin);
  irrecvs[2] = new IRrecv(irRRightPin);
  for (int i = 0; i < RECEIVERS; i++) {
    irrecvs[i]->enableIRIn();
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
}
void blinkC(int colorValueStartIndex) {
  analogWrite(red, colors[0][0]);
  analogWrite(green, colors[0][1]);
  analogWrite(blue, colors[0][2]);
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
  Serial.print(ps2x.Analog(PSS_LY), DEC);
  Serial.print(",");
  Serial.print(ps2x.Analog(PSS_LX), DEC);
  Serial.print(",");
  Serial.print(ps2x.Analog(PSS_RY), DEC);
  Serial.print(",");
  Serial.println(ps2x.Analog(PSS_RX), DEC);
  if (ps2x.ButtonPressed(PSB_CROSS)) {
    sendHit();
    Serial.println("send Hit");
  }
  if (ps2x.Analog(PSS_LY) < 125 || ps2x.Analog(PSS_LY) > 131) {
    drive(map(ps2x.Analog(PSS_LX), 0, 255, 0, 360));
  }
}

void sendHit() {
  for (int i  = 0; i < 3; i++) {
    irsend.sendSony(0x5A5 , 3);
    //Serial.println("send hit " + String(i));
    delay(200);
  }
}

int irRecv() {
  for (int i = 0; i < RECEIVERS; i++) {
    if ((!ledT) && (irrecvs[i]->decode(&results)))   // If RGB LED is off and we detect a signal
    {
      // This if statement is what checks for valid element codes from the receivers and ignores junk codes
      if ((results.value == waterCode) || (results.value == airCode) ||
          (results.value == grassCode) || (results.value == fireCode) ||
          (results.value == hitCode) || (results.value == earthCode) ||
          (results.value == elecCode)) {
        codeReceived = results.value;
        Serial.println(codeReceived);
      }
      else {
        Serial.println("IRpass");
      }
    }
  }
  switch (codeReceived) {
    case fireCode:
      blinkC(0);
      break;

    case waterCode:
      blinkC(1);
      break;

    case earthCode:
      blinkC(2);
      break;

    case airCode:
      blinkC(3);
      break;

    case elecCode:
      blinkC(4);
      break;

    case grassCode:
      blinkC(5);
      break;

    case hitCode:
      blinkC(6);
      break;

    default:  // default is to leave the RGB LED in last state
      break;
  }
}
void ledUpdate() {
  if ((ledT) && ((ledT + 5000) < millis())) {  // If light is on and 5 seconds passed
    analogWrite(red, 0);                     // Turn off RGB LED
    analogWrite(green, 0);
    analogWrite(blue, 0);
    ledT = 0;                                    // Reset Ton to zero in order to receive
    for (int i = 0; i < RECEIVERS; i++) {          // ReEnable the receivers when the LED goes off. Put here so that don't get buffer overflow in receiver.
      irrecvs[i]->resume();
    }
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
    ledUpdate();
    pidSteer();
    irRecv();
  }
}

void loop() {
  //ledUpdate();
  //readPS();
  // pidSteer();
  //turnAt();
  irRecv();
}
