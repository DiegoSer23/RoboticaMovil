#include "DualMC33926MotorShield.h"

boolean anterior = 0;    
boolean actual = 0; 
int contador = 0;  

DualMC33926MotorShield md;

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
  Serial.begin(112500);  
  pinMode(2,INPUT);    
   md.init();
}

 boolean debounce(boolean dato_anterior) 
 {

   boolean dato_actual = digitalRead(2);
   if (dato_anterior != dato_actual)
   {
     delay(10);
     dato_actual = digitalRead(2);
   }
   return dato_actual;
 }  

void loop() 
{           
  md.setM2Speed(400);
   md.
  actual = debounce(anterior); 
  if ( anterior == 0 && actual == 1) 
  {
         contador++;              
         
         delay (100);          
         Serial.println(contador);
  }
  
    anterior = actual; 
}