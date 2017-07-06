# Balancing the drone
the drone was balanced by PID controller. P-proportional I-integrel d-derivative.
derivative = if the gyro experiences a rate of change on the angle then the arduino will send power to the motors to spot this rotation. adjustments for this will cause the drone to stay at its current tilt.
proportional = if the drone tilts the gyro will return the angle of the tilt to the arduino and then the arduino will adjust the motors to fix the tilt in the drone. adjustments for this alone will cause the drone to rotate back and forth.
# Gyroscope Code
