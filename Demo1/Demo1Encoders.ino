
#define ENCODER_OPTIMIZE_INTERRUPTS
#include "DualMC33926MotorShield.h"
#include <Encoder.h>
#define Pi 3.1415926535897932384

DualMC33926MotorShield md;
Encoder knobLeft(3,5);
Encoder knobRight(2,6);
float positionL = 0.0;
float positionR = 0.0;
int motorSpeed = 0;

void setup() {
  Serial.begin(9600);
  md.init();
  md.setM1Speed(motorSpeed);

}

long positionLeft  = -999;
long positionRight = -999;


void loop() {

  if (motorSpeed <= 400){
    motorSpeed += 1;
  }
  md.setM1Speed(motorSpeed);

  long newLeft, newRight;
  newLeft = knobLeft.read();
  newRight = knobRight.read();
  if (newLeft != positionLeft || newRight != positionRight) {
    // converting counts to position, multiply counts*(pi/2)/(counts in pi/2 revs)
    positionL = (newLeft*(Pi/2.0))/800.0;
    positionR = (-1*newRight*(Pi/2.0))/800.0;
    Serial.print("Left: ");
    Serial.print(positionL);
        
    Serial.print(", Right: ");
    Serial.println(positionR);
    Serial.println();
    positionLeft = newLeft;
    positionRight = newRight;
  }

  // if a character is sent from the serial monitor,
  // reset both back to zero.
  if (Serial.available()) {
    Serial.read();
    Serial.println("Reset both knobs to zero");
    knobLeft.write(0);
    knobRight.write(0);
  }
}
