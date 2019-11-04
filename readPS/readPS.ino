#include <PS2X_lib.h>
#include <Servo.h>
#define PS2_DAT 7
#define PS2_CMD 6
#define PS2_SEL 5
#define PS2_CLK 4
#define pressures   false
#define rumble      false
PS2X psx;
Servo motorL;
Servo motorR;
int error = 0;
byte type = 0;
byte vibrate = 0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(300);
  error = psx.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);

}

void loop() {
  psx.read_gamepad();
  // put your main code here, to run repeatedly:
  Serial.print("Stick Values:");
  Serial.print(psx.Analog(PSS_LY), DEC); //Left stick, Y axis. Other options: LX, RY, RX
  Serial.print(",");
  Serial.print(psx.Analog(PSS_LX), DEC);
  Serial.print(",");
  Serial.print(psx.Analog(PSS_RY), DEC);
  Serial.print(",");
  Serial.println(psx.Analog(PSS_RX), DEC);

  motorL.write(map(psx.Analog(PSS_LY), 0, 255, 0, 360));
  motorR.write(map(psx.Analog(PSS_LY), 0, 255, 360, 0));

}
