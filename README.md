# RaspberryAnt
Using to Raspberry to drive a Hexapod

Note: This project is a work in progress.
About
This project is based around Python and two Raspberry Pi. The robot is a 6-legged (hexapod) robot with 3 degrees of freedom per leg. It is controlled with a converted PS2 controller

# Hardware

2 x Raspberry Pi
2 x Low-power Edimax USB wifi module used for connection to a home network
2 x XBee module used for connection between robot and the PS2 controller
2 x 16-Channel 12-bit PWM Drivers used for control of the servo PWM signals. This is based on the PCA9685 which has its own internal clock. 
18 x High Torque digital servos
1 x 2s1p Lipo Battery Pack
2 x 8-15A UBEC for converting the 8.4V from the Lipo Battery to 6V for the servos
1 x BEC for converting the 8.4V from the Lipo Battery to the raspberry
1 x Basomodel from Lynxmotion HR6
2 x fuse and holder. A 10A fuse was used
1 x 12V Toggle switch
1 x Lipo low voltage alarm

# Software

The following modules should be installed using apt-get after installing Raspbian on the Raspberry Pi:
Python 2.7
Adafruit Adafruit_PWM_Servo_Driver
Adafruit 17006-driver
OLED

Check out my YouTube channel for videos of the robot in action.

Best regards
PappaNiklas
