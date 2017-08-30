#include <Wire.h>
#include "mpu.h"


void setup(){
 
  Serial.begin(115200);
  Wire.begin();
  
  initialize_mpu();
}

void loop(){

}

void initialize_mpu(){
  
    //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x18); 
  Wire.write(0x18);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
}
