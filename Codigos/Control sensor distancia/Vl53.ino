#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;
const uint8_t maximumDistance = 800;///mm in mm , 800mm=80cm
const uint8_t type = 1;// 1=mm , 2= cm; 3=inch (1 mm = 0.03937 inch)
char *unit[]={"mm","cm","in"};// variable for unit, mm, cm or in

float distanceCm, distanceIn;

uint8_t distance;

void printOnSerial();

void setup() {
  // put your setup code here, to run once:
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.println("VLX53LOX test started.");
  Serial.begin(9600);


  sensor.init();
  sensor.setTimeout(500);
  sensor.startContinuous();
delay(1000);
}

void loop() {
   
 distance =sensor.readRangeContinuousMillimeters();
  if(type==3){    
    distanceIn =(float) (distance*0.03937);//convert distance to inch  
  }else if(type==2){     
    distanceCm =(float) (distance/10.0);//convert distanc to cm   
  }
  
  delay(500);
  printOnSerial();
}


void printOnSerial()
{
      Serial.print("Distance: ");  
        if(type==3){ 
           Serial.print(distanceIn);
        }else if(type==2){
           Serial.print(distanceCm);          
        }else{

           Serial.print(distance);         
        }
      Serial.print(unit[type-1]);      
      Serial.println();  
}
