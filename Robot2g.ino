/* Encoder Library - Basic Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */

#include <Encoder.h>

// Declaramos encoder del motorsote
Encoder myEnc(2, 3);

//Torque, el valor del torque es un valor flotante con signo que va de +-2047
//la salida es el DAC0, que después de los opams genera una señal de +-10v la cual lee el driver
//y envía la corriente hacia el motor. 
float torque=0;

void setup() {
   Serial.begin(115200);

  //Configuramos el motor en 0
  //pinMode(DAC0,OUTPUT);
  analogWriteResolution(12); // valores desde 0 a 4095
  analogWrite(DAC0,torque+2048);
  
}
//----------------------------------
//variables de posición
long oldPosition  = -999;
long currentDeg =0;
//objetivo en grados
float DesireDeg=90;
//----------------------------------
long prevT = 0;
int posPrev = 0;
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;
//----------------------------------
//variables de control
float kp=0.1;
float ki=0.1;
float kd=0.1;

//----------------------------------
float e=0;
float eintegral=0;
float eder=0;
float elast=0;
//----------------------------------
void loop() {
  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6;
  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    currentDeg = newPosition*360/5750;
    

 }
  

 //control
    e=DesireDeg-currentDeg;
    eintegral= eintegral+ e*deltaT;
    eder=(e-elast)/deltaT;
    
  float torque = kp*e + ki*eintegral +kd*eder;
    elast=e;
    prevT = currT;
    
  SetMotorsote(torque);
    Serial.print(newPosition);
    Serial.print("    DEG   " );
    Serial.println(currentDeg);
}
void SetMotorsote(float torque){
  if (torque>2047){
     analogWrite(DAC0,4095);
  }else if(torque<0){
     analogWrite(DAC0,0);
  }else{
    analogWrite(DAC0,torque+2048);
  }
  
}
