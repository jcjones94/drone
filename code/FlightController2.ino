




#include <Wire.h>
#include <RH_ASK.h>
#include <SPI.h>
#include <ServoTimer2.h>


int temperature;
int tmp;

int gyro_x, gyro_y, gyro_z;                                     //raw angle data from gyro(rate of change of angle)
long gyro_x_cal = 0, gyro_y_cal = 0, gyro_z_cal = 0;            //
long acc_x, acc_y, acc_z, acc_xz_vector, acc_yz_vector;         //raw accelerometer data

float angle_pitch = 0, angle_roll = 0;
float angle_pitch_acc, angle_roll_acc;
float angle_pitch_output = 0,  angle_roll_output = 0;
float pitch_sum = 0, roll_sum = 0;

float gyro_angle_con = 0.0001222*2*0.9;                         //converts gyro data to degrees
float gyro_angle_con_r = 0.00000213*2*0.9;                      //converts gyro data in rad to degrees
float acc_angle_correction = 0.06;

float power = 0;
float front, back, left, right;   //motor power varibles

ServoTimer2 m5; 
ServoTimer2 m6;
ServoTimer2 m9;
ServoTimer2 m10;

RH_ASK driver;
uint8_t buf[12];                    //reciever buffer
uint8_t buflen = sizeof(buf);
int analog_x, analog_y;
float analog_sen = 0.5/128.0;


//stablization varibles

float p_gain = 1;           
float d_gain = 0.03;          
float i_gain = 1.0/4.0;

float yaw_d_gain = 3.0/16.0;

float ccw = 1.0, cw = 1.0;

/***********
 *  SETUP
 ************/
void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(13, OUTPUT);
  m5.attach(5);
  m6.attach(6);
  m9.attach(9);
  m10.attach(10);

  setup_mpu_6050_registers();

  digitalWrite(13, LOW);                                               //turn off led to signal start of calibration
  
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++){                  //Run this code 2000 times                           
    read_mpu_6050_data();                                              //Read the raw acc and gyro data from the MPU-6050
    gyro_x_cal += gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
    delay(3);                                                          //Delay 3us to simulate the 250Hz program loop
  }
  gyro_x_cal /= 2000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyro_y_cal /= 2000;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gyro_z_cal /= 2000;

  digitalWrite(13, HIGH);                                              //turns on led to signal calibraiton complete
  
  if (!driver.init())                                                  //init 433mhz rf receiver
         Serial.println("init failed");//Use only for debugging
}

/************
 *  LOOP
 *************/
void loop() {

  get_angles();
  
  if (driver.recv(buf, &buflen)) {
       Serial.println("update");                                        //used for debugging
       analog_x = buf[1] - 128 - 5;
       analog_y = buf[2] - 128 + 2;
       tmp = 0;
  }else if(tmp >= 500 && buf[0] > 75){                                 //if no update is rec then drone lowers power to 75         
       buf[0]--;                                                        //so the drone doesnt fly away
  }

  if(power < buf[0]){                                                   //prevents power spikes
      power++;
  }else if(power > buf[0]){
      power--; 
  }

  pitch_sum += angle_pitch_output;                                             //sums the pitch and roll angle for the i_gain
  roll_sum += angle_roll_output;

  left  = power*4.0*ccw  - (angle_roll_output*p_gain  + gyro_y*d_gain  + roll_sum*i_gain  + yaw_d_gain*gyro_z) + 1000;       //left wing , 5
  right = power*4.0*ccw  + (angle_roll_output*p_gain  + gyro_y*d_gain  + roll_sum*i_gain  - yaw_d_gain*gyro_z) + 1000;       //right wing, 6
  back  = power*4.0*cw   + (angle_pitch_output*p_gain + gyro_x*d_gain  + pitch_sum*i_gain + yaw_d_gain*gyro_z) + 1000;       //back wing, 9
  front = power*4.0*cw   - (angle_pitch_output*p_gain + gyro_x*d_gain  + pitch_sum*i_gain - yaw_d_gain*gyro_z) + 1000;       //front wing, 10

  //user inputs from the analog stick on the controller
  left  += power*analog_sen*(analog_x - analog_y);
  right -= power*analog_sen*(analog_x - analog_y);
  back  -= power*analog_sen*(analog_x + analog_y);
  front += power*analog_sen*(analog_x + analog_y);
  
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

  Serial.print(angle_pitch); Serial.print("\t");
  Serial.print(angle_roll); Serial.print("\t");
  Serial.print(right); Serial.print("\t");
  Serial.println(left);

  tmp++;
}


/*
 * processes raw mpu data to angle data in degrees
 */
void get_angles(){
  read_mpu_6050_data();
  gyro_x -= gyro_x_cal;                                                //Subtract the offset calibration value from the raw gyro_x value
  gyro_y -= gyro_y_cal;                                                //Subtract the offset calibration value from the raw gyro_y value
  gyro_z -= gyro_z_cal;

  angle_pitch += gyro_x * gyro_angle_con;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll += gyro_y * gyro_angle_con;

  angle_pitch += angle_roll * sin(gyro_z * gyro_angle_con_r);               //If the IMU has yawed transfer the roll angle to the pitch angel
  angle_roll -= angle_pitch * sin(gyro_z * gyro_angle_con_r);               //If the IMU has yawed transfer the pitch angle to the roll angel

  acc_xz_vector = sqrt(acc_x*acc_x + acc_z*acc_z);
  acc_yz_vector = sqrt(acc_y*acc_y + acc_z*acc_z);

  angle_pitch_acc = asin((float)acc_y/acc_yz_vector)*57.29 + 0.57;
  angle_roll_acc = (asin((float)acc_x/acc_xz_vector) - acc_angle_correction)*-57.29;

  angle_pitch = angle_pitch * 0.999 + angle_pitch_acc * 0.001;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
  angle_roll = angle_roll * 0.999 + angle_roll_acc * 0.001;

  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;
}

/*
 * read raw data from mpu
 */
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

/*
 * mpu setup
 */
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
