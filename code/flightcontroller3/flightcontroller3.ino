
#include "I2Cdev.h"
#include <RH_ASK.h>
#include <SPI.h>
#include <ServoTimer2.h>

#include "MPU6050_6Axis_MotionApps20.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


MPU6050 mpu;   //initialize mpu class                             

#define INTERRUPT_PIN 2  // interrupt for mpu which is pin 2 for most arduinos
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


float power = 0;
float front, back, left, right;   //motor power varibles

ServoTimer2 m5; 
ServoTimer2 m6;
ServoTimer2 m9;
ServoTimer2 m10;

RH_ASK driver;
uint8_t buf[3];       //reciever buffer buf[power, analog_x, analog_y, rotation]
uint8_t buflen = sizeof(buf);
int analog_x, analog_y;
float analog_sen = 0.004;

//flight stabilization varibles
float p_gain = 300.0;       //adjusts the drone based on its angle  (d = derivative)
float d_gain = 7500.0;      //adjusts the drone based on its rate of in the angle (p = proportional)
float i_gain = 20.0;        //adjusts the drone based on its time spent at an angle (i = integral)
float x_sum = 0, y_sum = 0, z_sum = 0;      //sum of the angle used for i_gain
float gyro_x = 0.0 , gyro_y = 0.0, gyro_z = 0.0;    //used as previous mpu angle to calculate change in angle for d_gain 

float gyro_z_i = 0;         //adjusts the z angle to be zero at start up
float yaw_p_gain = 200.0;  //prevents drone rotation 
float ccw = 1.0, cw = 1.0;  //drone rotations

int analog_x_adj = -5;  //adjust if analog stick isn't zero at rest;
int analog_y_adj = 2;
float pitch_adj = -0.11;
float roll_adj = 0.02;

unsigned long timeLastRec;
bool set = false;
// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    m5.attach(5);
    m6.attach(6);
    m9.attach(9);
    m10.attach(3);

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    if (!driver.init())                                                  //init 433mhz rf receiver
         Serial.println("init failed");//Use only for debugging
    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    /*gets controller data on successful received and resets timeLastRec 
      if no signal is received for 5 seconds the drone resets all controller 
      data and decreases power for slow fall*/
    if (driver.recv(buf, &buflen)) {
       analog_x = buf[1] - 128 + analog_x_adj; //the analog stick is around 128 at rest
       analog_y = buf[2] - 128 + analog_y_adj; //so 128 is subtracted to make it 0 at rest
       timeLastRec = millis();
    }else if(timeLastRec < (millis() - 5000) && buf[0] > 80 || !dmpReady){          
       buf[0]--;
       analog_x = 0;
       analog_y = 0;                                                     
    }

    
    //prevents power from spiking
    if(power < buf[0]){                                                   
      power++;
    }else if(power > buf[0]){
      power--; 
    ]

    // wait for MPU interrupt but breaks is mpu Interrupt flag 
    while (!mpuInterrupt && fifoCount < packetSize) {
        /*
        if(mpu.getIntStatus() & 0x01){
            break;
        }*/
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
            
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        ypr[1] += pitch_adj; 
        ypr[2] += roll_adj;
        /*
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[2] * 180/M_PI);*/
        if(!set && millis() > 20000){
            gyro_z_i = ypr[0];  
            set = true;
            Serial.print("ready");
        }
        
        //as the drone rotates past pi it drops to -pi so when this happens 2pi is added to prevent the angle spike
        //same goes for if the drone is rotating the other way but instead 2pi is subtracted
        if((ypr[0] - gyro_z) > M_PI){
            gyro_z_i += 2*M_PI;  
        }else if((ypr[0] - gyro_z) < -M_PI){
            gyro_z_i -= 2*M_PI;  
        }

        //power is applied to the motors along with anglur correction data from the mpu
        //motor values have to be between 2000 and 1000
        left  = power*4.0*cw                                                
              - (ypr[2]*p_gain  + (ypr[2] - gyro_x)*d_gain + i_gain*x_sum 
              + (yaw_p_gain*(ypr[0] - gyro_z_i))) + 1000;       //left, 10
        right = power*4.0*cw  
              + (ypr[2]*p_gain  + (ypr[2] - gyro_x)*d_gain + i_gain*x_sum 
              - (yaw_p_gain*(ypr[0] - gyro_z_i))) + 1000;       //right , 9
        back  = power*4.0*ccw   
              + (ypr[1]*p_gain  + (ypr[1] - gyro_y)*d_gain + i_gain*y_sum 
              + (yaw_p_gain*(ypr[0] - gyro_z_i))) + 1000;       //back , 5
        front = power*4.0*ccw   
              - (ypr[1]*p_gain  + (ypr[1] - gyro_y)*d_gain + i_gain*y_sum 
              - (yaw_p_gain*(ypr[0] - gyro_z_i))) + 1000;       //front , 6
        
        //adds the analog stick data to motor values
        left  += power*analog_sen*(analog_x - analog_y);
        right -= power*analog_sen*(analog_x - analog_y);
        back  -= power*analog_sen*(analog_x + analog_y);
        front += power*analog_sen*(analog_x + analog_y);
          
        //prevtents motor values from dropping below 1000
        if(front < 1000)                                                  
            front = 1000;
        if(back < 1000)
            back = 1000;
        if(left < 1000)
            left = 1000;
        if(right < 1000)
            right = 1000;
        
        //sends motor values to ECS
        m5.write(back);
        m6.write(front);
        m9.write(right);
        m10.write(left);
        
        
        //saves the current angles to calculate angle change for next iteration
        gyro_y = ypr[1];
        gyro_x = ypr[2];
        gyro_z = ypr[0];
        
        //keeps running sum of the angles but only if drone is in flight which prevents the drone from
        //trying making angle corrections while resting on an uneven surface. also stops if user gives analog
        //input
        if(power > 60 
           && analog_x < 70 && analog_x > -70 
           && analog_y < 70 && analog_y > -70){
            
            x_sum += (ypr[2]);
            y_sum += (ypr[1]);
            z_sum += (ypr[0]);
        }
 

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}
