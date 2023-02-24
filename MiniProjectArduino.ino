#include <Wire.h>

#define SLAVE_ADDRESS 0x04

//encoder libraries and initialiations 
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#define Pi 3.1415926535897932384
Encoder knobLeft(2,3);
//Encoder knobRight(4,5);
float pos = 0.0;
int offset = 0;
int quadrant;

#include "DualMC33926MotorShield.h"
#include "math.h"

DualMC33926MotorShield ms;
// intital Kp value
double Kp = 13.71;//15.3786175942488;

// PID control variables
double Ke = .5;
double Ki = 1.93;//2.37803426483209;
double Kd = 0.0;

// comparative inputs and calculations
//max voltage
double umax = 7.5;
// voltage input
double u = 0.0;
// integral controller 
double I = 0.0;
double I_past = 0.0;

// past error
double e_past = 0.0;
// runnin timer?
double Ts = 0.0;
//Current time ASK LOCALIZATION ABOIUT THIS TO SEE how to read time
double Tc = millis()/1000;
// desired radians
double r = 0.0;
// current radians
double y = 0.0;
// current error
double e = 0.0;
//derivative scalar
double D = 0.0;
//voltage output
double V = 0.0;
double currentTime = 0.0;

bool motorGo = false;
void stopIfFault(){
  if(ms.getFault()){
    Serial.println("Fault");
    while(1);
  }
}

void setup() {
  pinMode(13, OUTPUT);
  Serial.begin(9600); // start serial for output 115200
  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);
  
  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

  ms.init();
  
  Serial.println("Ready!");
}

long positionLeft  = -999;
//long positionRight = -999;

void loop() {
  //r=3.14;
  // put your main code here, to run repeatedly:
  //delay(100);
  long newLeft, newRight;
  newLeft = knobLeft.read();
  //newRight = knobRight.read();
  //if (newLeft != positionLeft || newRight != positionRight) {
  if (newLeft != positionLeft) {
    //Serial.print("Position = ");
    // converseting counts to position, multiply counts*(pi/2)/(counts in pi/2 revs)
    pos = (newLeft*(Pi/2.0))/800.0;
    y = pos;
    //Serial.print(position);
   // Serial.print(", Right = ");
   // Serial.print(newRight);
    //Serial.println();
    positionLeft = newLeft;
    //positionRight = newRight;
  }
  


    //pi controller
    //currentTime = millis()/1000;
    Serial.print("r: ");
    Serial.println(r);

    //Serial.print("y: ");
    //Serial.println(y);
    
    e = r-y;

    //Serial.print("e: ");
    //Serial.println(e);
    if(e==0){
      I_past = 0;
      
    }
    //Serial.print(e);
    if (Ts>0){
      D = (e-e_past)/Ts;
      e_past = e;
    }  else{
      D=0;       
    }
    I = I_past + Ts*e;
    //Serial.print(I);
    u = Kp*e + Ki*I + Kd*D;
    //Serial.println(u);
    if (abs(u) > umax){
      if (u>=0){
        u = umax;
      } else{
        u = (-1)*umax; 
      }     
      if (e>=0){
        e = min(umax/Kp , abs(e));
      } else{
        e = (-1)*min(umax/Kp , abs(e));
      } 
     
      I = (u-Kp*e-Kd*D)/Ki;
  
    } 
    
    
    V = -u * (400/umax);
    //Serial.print(V);
    
    ms.setM1Speed(V);

    stopIfFault();
    currentTime = millis()/1000;
    Ts = currentTime - Tc;
    Tc = currentTime;
    I_past = I;
  }



void receiveData(int byteSize) {
  offset = Wire.read();
  //if offset byte recieved is 0, sending set point based on quadrant reading
  //if offset byte recieved is 1, reset encoder position
  r = 5;
  //Serial.print(offset);
  if(offset == 0){
    //Serial.write("quadrant: ");
    
      quadrant = Wire.read();
      //Serial.print(quadrant);
    
      r = (Pi/2) * (double(quadrant)-1);
      //Serial.print(double(r));
    
    
    

  }else if(offset ==1){
      knobLeft.write(0);
      y = 0.0;
      r=0.0;
      offset=0;
      //Serial.write("Reset Encoder");
      //motorGo = true;
  }
  //Serial.println(' ');
}


// callback for sending data
void sendData(){
  //Serial.print(position);
  //Wire.write(int(pos));
}


/*
void serialEvent(){
  if (Serial.available() > 0) {
    r = int(Serial.read() - '0')*(Pi/2);
    //Serial.print(data);
    DataRead = true;
  }
  Serial.flush();
}*/
