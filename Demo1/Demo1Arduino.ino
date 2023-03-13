#define ENCODER_OPTIMIZE_INTERRUPTS
#include "DualMC33926MotorShield.h"
#include <Encoder.h>
#include <Wire.h>
#define SLAVE_ADDRESS 0x04
#define Pi 3.1415926535897932384
#include "DualMC33926MotorShield.h"
#include "math.h"

DualMC33926MotorShield ms;


// intital Kp value

double Ke = .5;
double KpCon1 = 13.71;//15.3786175942488;

// PID control variables
double Kp1 = 48.3564;             //13.71;//15.3786175942488;
double Ki1 = 0.0;             //1.93;//2.37803426483209;
double Kd1 = 1.898917;

double Kp2 = 41.681;                //13.71;//15.3786175942488;
double Ki2 = 0.0;              // 1.93;//2.37803426483209;
double Kd2 = 2.623;

// comparative inputs and calculations
//max voltage
double umax = 7.5;
double phiDesired = 0.0;
// voltage delta and bar values
double VDelta = 0.0;

double VBar = 0.0;
double intialTime = 0.0;
int thresholdB = 0;
// integral controller 
double I1 = 0.0;
double I_past1 = 0.0;

double I2 = 0.0;
double I_past2 = 0.0;

double I3 = 0.0;
double I_past3 = 0.0;
// past error of controllers
double e_past1 = 0.0;

double e_past2 = 0.0;

double e_past3 = 0.0;
// runnin timer?
double Ts = 0.0;
//Current time 
double Tc = millis()/1000;
// desired angle turn
double desiredAngleTurn = 90.0;//-90;
// distance asked to go
double desiredFPS = 10.0;

// Encoder angle?
double actualPhi = 0.0;

double sentM1 = 0.0;
double sentM2 = 0.0;


//speed control this value we want as speed of motors potentially
double rho = 0.0;

// current error of controllers
double e1 = 0.0;
double e2 = 0.0;
double e3 = 0.0;
//derivative scalar
double D1 = 0.0;
double D2 = 0.0;
double D3 = 0.0;
//voltage output
double V1 = 0.0;
double V2 = 0.0;
int offset;

double currentTime = 0.0;

//DualMC33926MotorShield md;
Encoder knobLeft(3,5);
Encoder knobRight(2,6);
//float positionL = 0.0;
//float positionR = 0.0;
float angleL = 0.0;
float angleR = 0.0;
double phi = 0.0;
int motorSpeed = 0;
double r = 0.25;
double d = 0.9375;
bool doneTurning = false;

void setup() {
  Serial.begin(9600); //11500 or 9600?
  //md.init();
  //md.setM1Speed(motorSpeed);
  Serial.println("Dual MC33926 Motor Shield");
  ms.init();
  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);
  
  // define callbacks for i2c communication
  Wire.onReceive(receiveData);

}

void stopIfFault(){
  if(ms.getFault()){
    Serial.println("Fault");
    while(1);
  }
}

void receiveData(int byteSize) {
  offset = Wire.read();
  //if offset byte recieved is 0, sending set point based on quadrant reading
  //if offset byte recieved is 1, reset encoder counts
  //r = 5;
  //Serial.print(offset);
  if(offset == 0){
    //Serial.write("quadrant: ");
    
      //quadrant = Wire.read();
      //Serial.print(quadrant);
    
      //r = (Pi/2) * (double(quadrant)-1);
      //Serial.print(double(r));
    
    
    

  }else if(offset ==1){
      knobLeft.write(0);
      knobRight.write(0);
      //y = 0.0;
      //r=0.0;
      offset=0;
      phi = 0;
      //Serial.write("Reset Encoder");
      //motorGo = true;
      Serial.print("reset");
  }
  //Serial.println(' ');
}

long positionLeft  = -999;
long positionRight = -999;


void loop() {

  if (motorSpeed <= 400){
    motorSpeed += 1;
  }
  //md.setM1Speed(motorSpeed);

  long newLeft, newRight;
  newLeft = knobLeft.read();
  newRight = knobRight.read();
  if (newLeft != positionLeft || newRight != positionRight) {
    // converting counts to position, multiply counts*(pi/2)/(counts in pi/2 revs)
    angleL = (newLeft*(Pi/2.0))/800.0;
    angleR = (-1*newRight*(Pi/2.0))/800.0;

    phi = (r/d)*(angleL-angleR);
    //Serial.print("Left: ");
    //Serial.print(angleL);
        
    //Serial.print(", Right: ");
    //Serial.println(angleR);

    //Serial.print(phi);
    //
    //Serial.println();


    positionLeft = newLeft;
    positionRight = newRight;
  }
  //converts phi in radians to actual phi in degrees
  actualPhi = phi * 360/(2*Pi);
  //Serial.print("Actual phi ");
  //Serial.println(actualPhi);

  // if a character is sent from the serial monitor,
  // reset both back to zero.
  /*if (Serial.available()) {
    Serial.read();
    Serial.println("Reset both knobs to zero");
    knobLeft.write(0);
    knobRight.write(0);
  }*/


  
  
  //Controller Code VVVVVVVVV
   // put your main code here, to run repeatedly:
    /*e1 = desiredAngleTurn-actualPhi;

    if(e1==0){
      I_past1 = 0; 
    }
    if (Ts>0){
      D1 = (e1-e_past1)/Ts;
      e_past1 = e1;
      
    }  else{
      D1=0;
                            
    }
    I1 = 0;   //I_past1 + Ts*e1;
    
    //Serial.println(e1);
    //Serial.println(D1);
    //Serial.println(I1);
    phiDesired = Kp1*e1 + Ki1*I1 + Kd1*D1;
    //Serial.println(phiDesired);
    if (abs(phiDesired) > umax){
      if (phiDesired>=0){
        phiDesired = umax;
      } else{
        phiDesired = (-1)*umax; 
      }     
      if (e1>=0){
        e1 = min(umax/Kp1 , abs(e1));
      } else{
        e1 = (-1)*min(umax/Kp1 , abs(e1));
      }     
      I1 = 0.0; //(phiDesired-Kp1*e1-Kd1*D1)/Ki1;
    } */

    //Serial.println(phiDesired);

    
    //Controller 2
    
    e2 = desiredAngleTurn-actualPhi;//phiDesired
    //Serial.println(e2);
    if(e2==0){
      I_past2 = 0; 
    }
    if (Ts>0){
      D2 = (e2-e_past2)/Ts;
      e_past2 = e2;
      
    }  else{
      D2=0;
                            
    }
    I2 = 0.0 ;//I_past2 + Ts*e2;
    
   
    VDelta = Kp2*e2 + Ki2*I2 + Kd2*D2;
    //Serial.println(VDelta);
    
    if (abs(VDelta) > umax){
      if (VDelta>=0){
        VDelta = umax;
      } else{
        VDelta = (-1)*umax; 
      }     
      if (e2>=0){
        e2 = min(umax/Kp2 , abs(e2));
      } else{
        e2 = (-1)*min(umax/Kp2 , abs(e2));
      }     
      I2 = 0.0 ;//(VDelta-Kp2*e2-Kd2*D2)/Ki2;
    }

  //Controller 3
  rho = (r/2) * (angleL+angleR);//(360/(2*Pi));
  
  Serial.println(rho);
  if (doneTurning){
      e3 = desiredFPS-rho;
  
      if(e3==0){
        I_past3 = 0; 
      }
      if (Ts>0){
        D3 = (e3-e_past3)/Ts;
        e_past3 = e3;
        
      }  else{
        D3=0;
                              
      }
      I3 = 0.0 ;//I_past3 + Ts*e3;
      
     
      VBar = Kp2*e3 + Ki2*I3 + Kd2*D3;
      
      if (abs(VBar) > umax){
        if (VBar>=0){
          VBar = umax;
        } else{
          VBar = (-1)*umax; 
        }     
        if (e3>=0){
          e3 = min(umax/Kp2 , abs(e3));
        } else{
          e3 = (-1)*min(umax/Kp2 , abs(e3));
        }     
        I3 = 0.0 ;//(VBar-Kp2*e3-Kd2*D3)/Ki2;
      }
      VDelta = 0.0;
      if(abs(e3)<0.1){
        VDelta = 0;
        VBar = 0;
      }
  }
  
  
  
  
  V1 = (VDelta + VBar)/2;
  V2 = (VBar - VDelta)/2;

  //Serial.print("V1: ");
  //Serial.print(V1);
  //Serial.print(" ,V2");
  //Serial.print(V2);

  sentM1 = -1*(V1*250)/umax;
  sentM2 = (V2*250)/umax;
  //Serial.println(e3);
  if(abs(e2)<0.05){
    sentM1 = 0;
    sentM2 = 0;
    doneTurning = true;
  }
  


  ms.setM1Speed(sentM1);
  ms.setM2Speed(sentM2);

    stopIfFault();
    currentTime = millis()/1000;
    Ts = currentTime - Tc;
    Tc = currentTime;
    I_past1 = I1;
    I_past2 = I2;
    I_past3 = I3;
}
