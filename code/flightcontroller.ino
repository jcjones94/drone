#include <Wire.h>
#include <RH_ASK.h>
#include <SPI.h>
#include <ServoTimer2.h>

int gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;
float acc_x_i, acc_y_i, acc_z_i;
float acc_x_f, acc_y_f, acc_z_f;
float acc_x_adj = 0, acc_y_adj = 0, acc_z_adj = 0;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
int temperature;
float angle_pitch = 0, angle_roll = 0;
float angle_pitch_output = 0, angle_roll_output = 0;
float gyro_angle_calc = 0.0001222*2;
float gyro_angle_rad = 0.00000213*2;
float angle_roll_acc, angle_pitch_acc;
float analog_sen = 0.5/128.0;
float ccw = 1.0, cw = 1.05;
int analog_x, analog_y;
int temp = 0;
float power = 0;
float x_sum = 0, y_sum = 0;
int acc_adj = 0;

float front, back, left, right;


RH_ASK driver;

uint8_t buf[12];
uint8_t buflen = sizeof(buf);
float x, y;

ServoTimer2 m5;
ServoTimer2 m6;
ServoTimer2 m9;
ServoTimer2 m10;

float p_gain = 4/32.0;
float d_gain = 0.05/32.0;
float i_gain = 1/128;

float yaw_d_gain = 3.0/2192.0;

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(115200);
  pinMode(13,OUTPUT);
  m5.attach(5);
  m6.attach(6);
  m9.attach(9);
  m10.attach(10);
  setup_mpu_6050_registers();
  digitalWrite(13, LOW);
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++){                  //Run this code 2000 times                           
    read_mpu_6050_data();                                              //Read the raw acc and gyro data from the MPU-6050
    gyro_x_cal += gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
    acc_x_i += acc_x;
    acc_y_i += acc_y;
    acc_z_i += acc_z;
    delay(3);                                                          //Delay 3us to simulate the 250Hz program loop
  }
  gyro_x_cal /= 2000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyro_y_cal /= 2000;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gyro_z_cal /= 2000;
  acc_x_i /= 2000*50;
  acc_y_i /= 2000*50;
  acc_z_i /= 2000*50;
  
  digitalWrite(13,HIGH);
  if (!driver.init())
         Serial.println("init failed");//Use only for debugging
   
}

void loop() {
  // put your main code here, to run repeatedly:
  read_mpu_6050_data();
  gyro_x -= gyro_x_cal;                                                //Subtract the offset calibration value from the raw gyro_x value
  gyro_y -= gyro_y_cal;                                                //Subtract the offset calibration value from the raw gyro_y value
  gyro_z -= gyro_z_cal; 
  angle_pitch += gyro_x * gyro_angle_calc;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll += gyro_y * gyro_angle_calc;
  
  angle_pitch += angle_roll * sin(gyro_z * gyro_angle_rad);               //If the IMU has yawed transfer the roll angle to the pitch angel
  angle_roll -= angle_pitch * sin(gyro_z * gyro_angle_rad);               //If the IMU has yawed transfer the pitch angle to the roll angel

  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));
  
  angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;       //Calculate the pitch angle
  angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;

  angle_pitch = angle_pitch * 0.999 + angle_pitch_acc * 0.001;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
  angle_roll = angle_roll * 0.999 + angle_roll_acc * 0.001; 

  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;


  if (driver.recv(buf, &buflen)) {
       Serial.println("update");
       analog_x = buf[1] - 128;
       analog_y = buf[2] - 128;
       temp = 0;
  }else if(temp >= 500 && buf[0] > 75){
    buf[0]--;
  }

   if(power < buf[0]){
      power++;
   }else if(power > buf[0]){
      power--; 
   }
    x = angle_pitch_output;
    y = angle_roll_output;

    x_sum += x/250;
    y_sum += y/250;

    acc_x_f += acc_x;
    acc_y_f += acc_y;
    acc_z_f += acc_z;

    left  = power*4.0*ccw  -  power*(y*p_gain + gyro_y*d_gain + y_sum*i_gain + yaw_d_gain*gyro_z) + acc_z_adj + acc_x_adj + 1000;       //left , 5
    right = power*4.6*ccw  +  power*(y*p_gain + gyro_y*d_gain + y_sum*i_gain - yaw_d_gain*gyro_z) + acc_z_adj - acc_x_adj + 1000 + 30;  //right , 6
    back  = power*4.0*cw   +  power*(x*p_gain + gyro_x*d_gain + x_sum*i_gain + yaw_d_gain*gyro_z) + acc_z_adj + acc_y_adj + 1000;       //back , 9
    front = power*4.4*cw   -  power*(x*p_gain + gyro_x*d_gain + x_sum*i_gain - yaw_d_gain*gyro_z) + acc_z_adj - acc_y_adj + 1000;       //front , 10

    left  += power*analog_sen*(analog_x - analog_y);
    right -= power*analog_sen*(analog_x - analog_y);
    back  -= power*analog_sen*(analog_x + analog_y);
    front += power*analog_sen*(analog_x + analog_y);

    if(acc_adj == 10){
      acc_x_f /= 10.0*50.0;
      acc_y_f /= 10.0*50.0;
      acc_z_f /= 10.0*50.0;
      acc_z_adj = acc_z_i - acc_z_f + 330;
      acc_x_adj = acc_x_i - acc_x_f - 12;
      acc_y_adj = acc_y_i - acc_y_f - 4;
      acc_adj = 0;
      acc_x_f = 0;
      acc_y_f = 0;
      acc_z_f = 0;
    }

    if(front < 1000)
      front = 1000;
    if(back < 1000)
      back = 1000;
    if(left < 1000)
      left = 1000;
    if(right < 1000)
      right = 1000;

    m5.write(left);
    m6.write(right);
    m9.write(back);
    m10.write(front);

  /*
  Serial.print(acc_x); Serial.print("\t");
  Serial.print(acc_y); Serial.print("\t");
  Serial.print(acc_z); Serial.print("\t");
  Serial.print(temperature); Serial.print("\t");
  */
  Serial.print(acc_x_adj); Serial.print("\t");
  Serial.println(acc_y_adj);
  temp++;
  acc_adj++;

}
void read_mpu_6050_data(){                                             //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68,14);                                           //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);                                        //Wait until all the bytes are received
  acc_x = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_x variable
  acc_y = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_y variable
  acc_z = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_z variable
  temperature = Wire.read()<<8|Wire.read();                            //Add the low and high byte to the temperature variable
  gyro_x = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_x variable
  gyro_y = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_y variable
  gyro_z = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_z variable

}
void setup_mpu_6050_registers(){
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the accelerometer (+/-8g)
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x18); 
  Wire.write(0x10);                                                     //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
}
