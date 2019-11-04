#include <IRremote.h>
#define RECEIVERS 3
#define irreceiveLeftPin A4
#define irreceiveFrontPin A0
#define irreceiveRightPin 10
IRrecv *irrecvs[RECEIVERS];
decode_results results;
IRsend irsend;
void setup() {
  Serial.begin(9600);
  delay(2000);
  Serial.println("init");
  irrecvs[1] = new IRrecv(irreceiveFrontPin);
  for (int i = 0; i < RECEIVERS; i++) {
    irrecvs[i]->enableIRIn();
  }
}
void loop() {
  for (int i  = 0; i < 3; i++) {
    irsend.sendSony(0x5A5 , 3);
    Serial.println("send hit " + String(i));
    delay(100);
  }
  for (int i = 0; i < RECEIVERS; i++) {
    irrecvs[i]->enableIRIn();
    delay(40);
  }
  for (int i = 0; i < 2; i++) {
    Serial.println ("get results");
    //    if (irrecvs[i]->decode(&results))    {
    irrecvs[i]->decode(&results);
    Serial.println("decode");
    Serial.println(results.value, HEX);
    if (irrecvs[i] = 0xB13) {
      Serial.println("water");
    } else if (irrecvs[i] = 0xC9A) {
      Serial.println("grass");
    } else if (irrecvs[i] = 0xEA9) {
      Serial.println("earth");
    } else if (irrecvs[i] = 0xA19) {
      Serial.println("air");
    } else if (0xE1E) {
      Serial.println("electricty");
    } else if (0xF19) {
      Serial.println("fire");
    } else if (0x5A5) {
      Serial.println("hit");
    }
    irrecvs[i]->resume();
  }
}
