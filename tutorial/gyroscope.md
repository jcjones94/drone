# Balancing the drone
The drone is balanced by PID controller. P-proportional I-integral d-derivative.

## Derivative
If the gyroscope experiences a rate of change on the angle, then the arduino will send power to the motors to spot this rotation.  Adjustments for this will cause the drone to stay at its current tilt.

## Proportional 
If the drone tilts, the gyroscope will return the angle of the tilt to the arduino.  Then the arduino will adjust the motors to fix the tilt in the drone.  Adjustments for this alone will cause the drone to rotate back and forth.

## Derivative

# Gyroscope Code
