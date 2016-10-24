# protobot-2016
2016 off-season code development

Goals for the offseason:  find a stable gyro and develop a library for it
                          develop a vision option
                          develop repeatable "step" code for autonomous mode

Pixy.cpp and Pixy.h are a library for using a Pixy camera via the I2C bus on the roboRIO.

BNO055.cpp and BNO055.h are a library for using the Adafruit BN0055 breakout board via I2C.

Profile.cpp and Profile.h are a library for implementing "drive profiles" in autonomous mode.
Requires a gyro and encoder.  Also requires the PID.cpp and PID.h files.

