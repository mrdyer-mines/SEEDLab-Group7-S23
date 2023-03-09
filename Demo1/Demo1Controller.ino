#include "DualMC33926MotorShield.h"
#include "math.h"

DualMC33926MotorShield ms;
// intital Kp value

double Ke = .5;
double KpCon1 = 13.71;//15.3786175942488;

// PID control variables
double Kp1 = 0.0;             //13.71;//15.3786175942488;
double Ki1 = 0.0;             //1.93;//2.37803426483209;
double Kd1 = 0.0;

double Kp2 = 0.0;                //13.71;//15.3786175942488;
double Ki2 = 0.0;              // 1.93;//2.37803426483209;
double Kd2 = 0.0;

// comparative inputs and calculations
//max voltage
double umax = 7.5;
double phiDesired = 0.0;
// voltage delta and bar values
double VDelta = 0.0;

double VBar = 0.0;
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
double desiredAngleTurn = 0.0;
// distance asked to go
double desiredFPS = 0.0;

// Encoder angle?
double actualPhi = 0.0;


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

double currentTime = 0.0;


// what pins to match up to the comapring values here
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Dual MC33926 Motor Shield");
  ms.init();
}

void stopIfFault(){
  if(ms.getFault()){
    Serial.println("Fault");
    while(1);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
    e1 = desiredAngleTurn-actualPhi;

    if(e1==0){
      I_past1 = 0; 
    }
    if (Ts>0){
      D1 = (e1-e_past1)/Ts;
      e_past1 = e1;
      
    }  else{
      D1=0;
                            
    }
    I1 = I_past1 + Ts*e1;
    
   
    phiDesired = Kp1*e1 + Ki1*I1 + Kd1*D1;
    
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
      I1 = (phiDesired-Kp1*e1-Kd1*D1)/Ki1;
    } 
    //Controller 2

    e2 = phiDesired-actualPhi;

    if(e2==0){
      I_past2 = 0; 
    }
    if (Ts>0){
      D2 = (e2-e_past2)/Ts;
      e_past2 = e2;
      
    }  else{
      D2=0;
                            
    }
    I2 = I_past2+ Ts*e2;
    
   
    VDelta = Kp2*e2 + Ki2*I2 + Kd2*D2;
    
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
      I2 = (VDelta-Kp2*e2-Kd2*D2)/Ki2;
    }

//Controller 3
if ((e2 <= 0.01)&&(e2 >= -0.01)){
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
    I3 = I_past3 + Ts*e3;
    
   
    VBar = Kp2*e3 + Ki2*I3 + Kd2*D3;
    
    if (abs(VBar) > umax){
      if (VBar>=0){
        VBar = umax;
      } else{
        VBar = (-1)*umax; 
      }     
      if (e2>=0){
        e3 = min(umax/Kp2 , abs(e3));
      } else{
        e3 = (-1)*min(umax/Kp2 , abs(e3));
      }     
      I3 = (VBar-Kp2*e3-Kd2*D3)/Ki2;
    }
}

    V1 = (VDelta + VBar)/2;
    V2 = (VBar - VDelta);

  ms.setM1Speed(V1);
  ms.setM2Speed(V2);

    stopIfFault();
    currentTime = millis()/1000;
    Ts = currentTime - Tc;
    Tc = currentTime;
    I_past1 = I1;
    I_past2 = I2;
    I_past3 = I3;

}


// Checking the error of the first two controllers and then going into the third controller to make the 
// robot turn and then go straight. Findiong the angle is the comparison of the actual angle of the encoders
