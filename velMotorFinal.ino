#include "DualMC33926MotorShield.h"
//#include <Pid.h>
//#include <Phpoc.h>
//#include <PhpocExpansion.h>
DualMC33926MotorShield md;
int i=0; //Velocidad de entrada
int o=0, k=0; //Velocidad del motor
int los = 0; //Velocidad en PWM
float a = 0; //Corriente miliamps que consume
float arel=0; //Corriente que consume por el PWM
double Kp=2, Ki=5, Kd=1;
PID myPID(i, los, 100, Kp, Ki, Kd, DIRECT);
void stopIfFault()
{
  if (md.getFault())
  {
    Serial.println("fault");
    while(1);
  }
}

void setup()
{
  Serial.begin(9600);
  Serial.println("Dual MC33926 Motor Shield");
  Serial.println("Velocidad del motor:");
  md.init();
  //myPID.SetMode(AUTOMATIC);
}

void loop()
{
    if (Serial.available() > 0) {
    // read the incoming byte:
    i = Serial.parseInt();
    if(i>23 || i<0){   
    }
    else{
      Serial.println("Velocidad Deseada:");
      stopIfFault();
      los = 18.182 * i;
      if(los < 0){
        los = 0;
      }
      if(los > 400){
        los = 400;
      }
      Serial.print("PWM del motor:");
      Serial.println(los);
      md.setM1Speed(los);
      a = md.getM1CurrentMilliamps();
      while(los*0.00027 != a*0.0049){
      if(los*0.00027 < a*0.0049){
        los++;
      }
        else{
          if(los*0.00027 > a*0.0049){
            los--;
             }
          }
          //Serial.println(los);
          
      }
      k=los/18.17;
      while(k!=i){
        if(k<i){
          k++;
        }
        else{
          k--;
        }
    delay(2);
    }
        Serial.print("Actual Speed:");
        o=k;
        Serial.println(o);
    }
    k=0;
    }
}
