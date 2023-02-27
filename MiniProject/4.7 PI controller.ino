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
