
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#define Pi 3.1415926535897932384
Encoder knobLeft(2,3);
//Encoder knobRight(4,5);
float position = 0.0;

void setup() {
  Serial.begin(9600);
  //Serial.println("TwoKnobs Encoder Test:");
}

long positionLeft  = -999;
//long positionRight = -999;

void loop() {
  long newLeft, newRight;
  newLeft = knobLeft.read();
  //newRight = knobRight.read();
  //if (newLeft != positionLeft || newRight != positionRight) {
  if (newLeft != positionLeft) {
    Serial.print("Position = ");
    // converseting counts to position, multiply counts*(pi/2)/(counts in pi/2 revs)
    position = (newLeft*(Pi/2.0))/800.0;
    Serial.print(position);
   // Serial.print(", Right = ");
   // Serial.print(newRight);
    Serial.println();
    positionLeft = newLeft;
    //positionRight = newRight;
  }
  // if a character is sent from the serial monitor,
  // reset both back to zero.
  if (Serial.available()) {
    Serial.read();
    Serial.println("Reset both knobs to zero");
    knobLeft.write(0);
    //knobRight.write(0);
  }
}