#include <IRremote.h>
IRrecv *irrecvs[RECEIVERS];
decode_results results;

void setup() {
IRrecv irrecv(A0)
irrecvs[i].enableIRIn();
}

void loop() {
  // put your main code here, to run repeatedly:
if (irrecv.decode(&results)) {
    Serial.println(results.value, HEX);
    irrecv.resume(); // Receive the next value
  }
  delay(100);
}
}
