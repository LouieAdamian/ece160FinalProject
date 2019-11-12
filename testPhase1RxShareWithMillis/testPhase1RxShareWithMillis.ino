/*********************************************
  testPhase1RxShare
  Written by: Mario Simoni
  Date: 11/4/2019

  The purpose of this file is to demonstrate how to read from 3 different IR Receivers without
  having contention between the receivers. A problem arises if one receiver reads junk after
  another receiver receives a correct code. The junk can overright the correct code in the
  "decode_results" object "results".

  Each time a receiver is read using "if (irrecvs[i]->decode(&results))" it only stores the "results"
  value into the variable "codeReceived" if it is a valid code. If it is junk, the "codeReceived" value
  is unchanged. Each loop iteration, the "codeReceived" value must be reinitialized to 0.

  You must also be sure to use the <IRremote.h> from the Arduino-IRremote-multi-receiverFile library.

*/

#include <IRremote.h>

// Initialize all of the codes for the different elements
#define fireCode 0xF19
#define waterCode 0xB13
#define grassCode 0xC9A
#define earthCode 0xEA9
#define airCode 0xA19
#define elecCode 0xE1E
#define hitCode 0x5A5

// Setup the correct pin numbers for the IR receivers and create the receiver objects
// Note that while there are three IRrecv objects, one for each sensor, there is only
// one decode_results object. This is what causes one sensor to overright the value
// from another sensor.
#define RECEIVERS 3
#define irreceiveLeftPin 5
#define irreceiveFrontPin 6
#define irreceiveRightPin 7

IRrecv *irrecvs[RECEIVERS];
decode_results results;

//RGB LED Pins
#define rgbRedPin 8
#define rgbBluePin 9
#define rgbGreenPin 10

// define color values in groups of three RGB values for: orange, green, brown (purple), white, yellow, blue, red
const short colors[21] = {255, 55, 0,
                          0, 255, 0,
                          128, 20, 128,
                          255, 255, 255,
                          128, 128, 0,
                          0, 64, 128,
                          128, 0, 0
                         };

// These phrases represent the index value in the colors[] array where the 3 RGB values start
// for that color
#define orangeStart 0
#define greenStart 3
#define brownStart 6
#define whiteStart 9
#define yellowStart 12
#define blueStart 15
#define redStart 18

unsigned long ledTon = 0;

void setup() {

  Serial.begin(9600);

  // Make sure the RGB LED is off when program starts
  analogWrite(rgbRedPin, 0);
  analogWrite(rgbGreenPin, 0);
  analogWrite(rgbBluePin, 0);

  // Initialize and enable the IR reciever modules
  irrecvs[0] = new IRrecv(irreceiveLeftPin); // Receiver #0: Left
  irrecvs[1] = new IRrecv(irreceiveFrontPin); // Receiver #1: pin 3
  irrecvs[2] = new IRrecv(irreceiveRightPin); // Receiver #2: pin 4

  for (int i = 0; i < RECEIVERS; i++)
    irrecvs[i]->enableIRIn();

}  // End of setup() function

/*
   void loop()
   Each time you go through the loop, check for a received code and then light the RGB with the appropriate color
*/

void loop() {
  // must initialize the codeReceived value each time through the loop so that the light won't keep lighting
  int codeReceived = 0;

  if ((ledTon) && ((ledTon + 5000) < millis())) {  // If light is on and 5 seconds passed
    analogWrite(rgbRedPin, 0);                     // Turn off RGB LED
    analogWrite(rgbGreenPin, 0);
    analogWrite(rgbBluePin, 0);
    ledTon = 0;                                    // Reset Ton to zero in order to receive
    for (int i = 0; i < RECEIVERS; i++) {          // ReEnable the receivers when the LED goes off. Put here so that don't get buffer overflow in receiver.
      irrecvs[i]->resume();
    }
  }

  for (int i = 0; i < RECEIVERS; i++) {
    if ((!ledTon) && (irrecvs[i]->decode(&results)))   // If RGB LED is off and we detect a signal
    {
// This if statement is what checks for valid element codes from the receivers and ignores junk codes
      if ((results.value == waterCode) || (results.value == airCode) ||
          (results.value == grassCode) || (results.value == fireCode) ||
          (results.value == hitCode) || (results.value == earthCode) ||
          (results.value == elecCode)) {
        codeReceived = results.value;
      }
    }
  }

  // This switch-case statement lights the RGB LED based on the codeReceived value
  switch (codeReceived) {
    case fireCode:
      blinkRGB(orangeStart);
      break;

    case waterCode:
      blinkRGB(blueStart);
      break;

    case earthCode:
      blinkRGB(brownStart);
      break;

    case airCode:
      blinkRGB(whiteStart);
      break;

    case elecCode:
      blinkRGB(yellowStart);
      break;

    case grassCode:
      blinkRGB(greenStart);
      break;

    case hitCode:
      blinkRGB(redStart);
      break;

    default:  // default is to leave the RGB LED in last state
      break;
  }

} // End of loop() function


// Turns the RGB LED on with the color corresponding to colorValueStartIndex and sets the Ton time 
void blinkRGB(int colorValueStartIndex) {
  analogWrite(rgbRedPin, colors[colorValueStartIndex]);
  analogWrite(rgbGreenPin, colors[colorValueStartIndex + 1]);
  analogWrite(rgbBluePin, colors[colorValueStartIndex + 2]);
  ledTon = millis();
}
