#include <Wire.h>


int gyro_x, gyro_y, gyro_z;
int acc_x, acc_y, acc_z;


void setup(){
 
  Serial.begin(115200);
  Wire.begin();
 
 
  // INTERRUPT DETECTION ROUTINE
  volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
  void dmpDataReady() {
    mpuInterrupt = true;
  }
 
 
  initialize_mpu();
 
}

void loop(){
  read_mpu()
}


void read_mpu(){
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x74);                                                    //Send the requested starting register
  Wire.write(0x0c);    
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68,12);                                           //Request 14 bytes from the MPU-6050
  
  acc_x = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_x variable
  acc_y = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_y variable
  acc_z = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_z variable
  gyro_x = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_x variable
  gyro_y = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_y variable
  gyro_z = Wire.read()<<8|Wire.read();      

}


void initialize_mpu(){
  
    //selects x_gyro as clock reference 
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x01);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  
 
 //sets gyro full scale range to +/-2000 dps and acc full scale range to +/- 16g
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x18); 
  Wire.write(0x18);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
 
 
 //selects which data types get queued into the fifo buffer (gyro and acc data), 12 bytes of data
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x23);                                                    //Send the requested starting register
  Wire.write(0x78);                                           
  Wire.endTransmission(); 
 
 //sets the interrupt singal to occurre when the fifo buffer has new data to be read
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x38);                                                    //Send the requested starting register
  Wire.write(0x01);                                           
  Wire.endTransmission(); 
}
