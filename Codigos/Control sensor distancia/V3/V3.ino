#include "Adafruit_VL53L0X.h"

int t=0;
int periodo = 40;  
unsigned long tiempoAnterior = 0;  

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void setup() {
  Serial.begin(115200);
  
  while (! Serial){delay(1);}

  Serial.println("Adafruit VL53L0X test.");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
  
  Serial.println(F("VL53L0X API Continuous Ranging example\n\n"));
  lox.startRangeContinuous();
}

void loop() {
    t = millis();
     if(t - tiempoAnterior >= periodo)
     { 
     
       if (lox.isRangeComplete()) {
        Serial.print(-0.0009*lox.readRange()+0.1822,4);
        Serial.print(" ");
        Serial.println( t);}

        tiempoAnterior = t;

     }
     
}
