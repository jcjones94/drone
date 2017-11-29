# drone
drone controlled by arduino

working on building a quadcopter using a arduino and mpu 6050 gyro/acc as the flight controller. 
Im currently using 433Mhz rf transmitter and recievers for communcation between the drone and controller.
The controller is built using an potentiometer for power and playstation analog stick for steering

Libraries:
  Using Radiohead for rf transmitting and receiving. Using ServoTimer2 for controlling the power sent to the motors. 
  Im using ServoTimer2 because radiohead and servo.h have conflicting libraries so only one can be used. 
  wire.h for connecting to MPU6050. Currently using my own code for communicating with the MPU6050 because it allows me to change the 
  sensitivity of the gyro/acc

Arduino Connections:

  drone Arduino
  
    digital output pins
    1
    2
    3
    4
    5     left wing esc
    6     right wing esc
    7 
    8
    9     back wing esc
    10    front wing esc
    11    rf receiver data line
    12   
    13    initial  calibration led (on = calibrating, off = ready for flight)
    
    Analog Read
    A0
    A1
    A2
    A3
    A4    SCL on mpu 6050 gyro/acc
    A5    SDA on mpu 6050 gyro/acc
    
  controller Arduino
  
    pin 12   rf transmitter data line
    pin 13   off/on led
    A0       potentiometer
    A1       X axis of analog stick
    A2       Y axis of analog stick
     
    ![drone_flying](download.jpg)
