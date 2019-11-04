#include <Psx_analog.h>
//#include <Servo.h>
//Servo rWheel;
//Servo lWheel;
Psx psx
void setup() {
  psx.setupPins(4,5,6,7);
  psx.initcontroller(psxAnalog);
  Serial.begin(9600);
//  lWheel.attach(13);//180 is forward
//  rWheel.attach(12);// 0 is forward
}
char c;
void loop() {
//  if (Serial.available() > 0) {
//    c = Serial.read();
//    Serial.println(c);
//  }
//  if (c == 'w' || c == 'W') {
//    lWheel.write(180);
//    rWheel.write(0);
//  }
//  if ( c == 'a' || c == 'A') {
//    lWheel.write(30);
//    rWheel.write(0);
//  }
//  if (c == 's' || c == 'S') {
//    lWheel.write(0);
//    rWheel.write(180);
//  }
//  if (c == 'd' || c == 'D') {
//    lWheel.write(180);
//    rWheel.write(45);
//  }
//  if (c == '0') {
//    rWheel.write(90);
//    lWheel.write(90);
//  }
//  // lWheel.write(180);
//  // rWheel.write(0);
//  delay(1000);
//  c = 0;
 psx.poll();                                      // Psx.read() initiates the PSX controller and returns
  Serial.print("\n");                                            // the button data
  Serial.print(psx.Controller_mode, HEX);     // prints value as string in hexadecimal (base 16) 
  Serial.print(psx.digital_buttons, HEX);     // prints value as string in hexadecimal (base 16)  
  Serial.print(psx.Right_x, HEX);     // prints value as string in hexadecimal (base 16)    
  Serial.print(psx.Right_y, HEX);     // prints value as string in hexadecimal (base 16)    
  Serial.print(psx.Left_x, HEX);     // prints value as string in hexadecimal (base 16)    
  Serial.print(psx.Left_y, HEX);     // prints value as string in hexadecimal (base 16)  
//  if(psx.Right_x() > 1000 || psx.Right_x() < 1048){
//    
//  }
//  else{
//    map
//  }
}

void driveMotors() {

}
