//Vl53l0x library

#include <Wire.h>
#include "Adafruit_VL53L0X.h"
Adafruit_VL53L0X lox = Adafruit_VL53L0X();


//definition name of the GPIO
const int A = D4; //Channel A encoder
const int Button = D3; //Reset position button 
const int B = D5;//Channel B encoder
const int Pw = D6; //PWM ouput
const int DirA = D8; // Direction output - left
const int DirB = D7; // Direction output - right

//Slave control variables
float ppr = 4180; //number of points per turn 
float pi = 3.141592;
float p2pos =(2*pi)/ppr; //conversion factor to radians
float Pos_s = 0; //current motor position
float ref_s  = 0; //Motor reference in radians
float c_s = 0; //PID output
float c_o = 0; //PWM output
float err_s = 0; // Reference error relative to the current position
float err_d_s = 0; //derivative error
float err_s_s = 0; //integral error
float err_a_s = 0; //previous mistake
float Kp_s = 79.2;//18.8 proportional constant
float Kd_s = 1.03;//0.2124 derivative constant
float Ki_s = 0; //integral constant
int t_s = 4; //execution time of the slave loop in ms (sampling time)
int t_a_s = 0; //previous time
float ts_s = 0.004; //sampling time seconds
float its_s = 250; //inverse of sampling time

//Mater control variables
float c_m = 0; //PID output
float Pos_m = 0; //current motor position
float ref_m  = 0; //ball reference in meters
float err_m = 0; // Reference error relative to the current position
float err_d_m = 0; //derivative error
float err_s_m = 0; //integral error
float err_a_m = 0; //previous mistake
float Kp_m = 0.935; //0.935 proportional constant
float Kd_m = 0.195;//0.035 //derivative constant
float Ki_m = 0.075; //integral constant
int t_m = 40; //execution time of the master loop in ms (sampling time)
float ts_m = 0.04; //sampling time seconds
int t_a_m = 0; //previous time
float its_m = 25;  //inverse of sampling time



//general variables
int timems = 0;
long int b = 0; //encoder reading


// Account reset button programming
ICACHE_RAM_ATTR void ButtonPress()
  {
   if(digitalRead(Button) == LOW)
   {b = 0;}  
  }


// Programming the interruptions for reading the channel A encoder
ICACHE_RAM_ATTR void ARise()
  {
   if(digitalRead(B) == digitalRead(A))
   {b--;}
   else
   {b++;}   
  }
  
// Programming the interruptions for reading the channel B encoder
ICACHE_RAM_ATTR void BRise()
  {
   if(digitalRead(A) != digitalRead(B))
   {b--;}
   else
   {b++;}   
  }

void setup() {
  analogWriteRange(2000);
  Serial.begin(115200); //Serial communication speed
  pinMode(Button, INPUT_PULLUP); //position reset button definition

//distance sensor initialization
  while (! Serial){delay(1);}
  Serial.println("Adafruit VL53L0X test.");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
  lox.startRangeContinuous();

//initialization of GPIOs and interrupts
  attachInterrupt(digitalPinToInterrupt(Button), ButtonPress, FALLING);
  pinMode(A, INPUT_PULLUP);
  pinMode(B, INPUT_PULLUP);
  pinMode(DirA, OUTPUT);
  pinMode(DirB, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(A), ARise, RISING);
  attachInterrupt(digitalPinToInterrupt(B), BRise, RISING);
}

void loop() { 

  timems = millis(); 

////////////////////////////////////////////Master loop//////////////////////////////////////////////////
 
  if(timems - t_a_m >= t_m){

      if (timems >30000) {ref_m = 0.05;}
      
      if (timems >45000) {ref_m = 0.1;}
   
      if (timems >60000) {ref_m = 0.05;}
  
      if (timems >75000) {ref_m = 0;}
  
      if (timems >90000) {ref_m = -0.05;} 

      if (timems >105000) {ref_m = -0.1;}

      if (timems >120000) {ref_m = -0.05;}

      if (timems >135000) {ref_m = 0;}
   /*  */
  
    //Distance sensor reading and error calculation
    if (lox.isRangeComplete()) {Pos_m = -0.0009*lox.readRange()+0.1822;}
    err_m = ref_m - Pos_m;
    
    //PID
    err_d_m = (err_m - err_a_m)*its_m;
    err_s_m = (err_s_m + err_m);
    c_m = err_m * Kp_m + err_d_m * Kd_m + err_s_m*Ki_m*(float)ts_m;
    err_a_m = err_m;

    //Angle limiters
    if(c_m > pi/16){
      c_m = pi/16;
    }
    else if (c_m < -pi/16){
      c_m = -pi/16;
    }

    //slave loop reference update
    ref_s = c_m;
    t_a_m = millis();    
  }



////////////////////////////////////////////Slave loop//////////////////////////////////////////////////

    if(timems - t_a_s >= t_s)
  {

    //encoder reading and error calculation
    Pos_s = b * p2pos;
    err_s = ref_s - Pos_s;

    //PID
    err_d_s = (err_s - err_a_s)*its_s;
    err_s_s = (err_s_s + err_s);
    c_s = err_s * Kp_s + err_d_s * Kd_s*+ err_s_s*Ki_s*(float)ts_s;
    err_a_s = err_s;

    //voltage limiters
    if(c_s > 12){
      c_s = 12;
    }
    else if (c_s < -12){
      c_s = -12;
    }
    
    //voltage to PWM conversion
    c_o = (c_s * 1024)/12;
    c_o = ChangeDir((int)c_o);
    analogWrite(Pw,c_o);

    //printing of the data in the serial
    Serial.print(timems);
    Serial.print(",");
    Serial.print(Pos_m,5);
    Serial.print(",");
    Serial.print(ref_m,3);
    Serial.print(",");
    Serial.print(err_m,3);
    Serial.print(",");
    Serial.print(ref_s,3);
    Serial.print(",");
    Serial.print(Pos_s,3);
    Serial.print(",");
    Serial.print(err_s,3);
    Serial.print(",");
    Serial.println(c_s,3);
    
    t_a_s = millis();    
  }
}


//direction change function
int ChangeDir(int P)
{
  if(P > 0)
  {
    digitalWrite(DirA,LOW);  
    digitalWrite(DirB,HIGH); 
    
  }
  else
  {
    digitalWrite(DirB,LOW);
    digitalWrite(DirA,HIGH); 
    P = P*-1; 
  }
  return P;
}
