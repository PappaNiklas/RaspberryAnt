# RaspberryAnt
Using to Raspberry to drive a Hexapod

Note: This project is a work in progress.

# About
This project is based around Python and two Raspberry Pi. The robot is a 6-legged (hexapod) robot with 3 degrees of freedom per leg. It is controlled with a converted PS2 controller

# Hardware sender
-  An old broken LyxMotion PS2-copy
-  OLED ssd1306 for the 0.96" or sh1106 for the 1.3"
-  1 x Raspberry Pi 1B
-  1 x Low-power Edimax USB wifi module used for connection to a home network
-  1 x XBee module used for connection between robot and the PS2 controller


# Hardware receiver

-  1 x Raspberry Pi 2B
-  1 x Low-power Edimax USB wifi module used for connection to a home network
-  1 x XBee module used for connection between robot and the PS2 controller
-  2 x 16-Channel 12-bit PWM Drivers used for control of the servo PWM signals. This is based on the PCA9685 which has its own internal clock. 
-  18 x High Torque digital servos Hitec HS-5645MG
-  1 x 2s1p Lipo Battery Pack
-  2 x 8-15A UBEC for converting the 8.4V from the Lipo Battery to 6V for the servos, 10A fuse
-  1 x BEC for converting the 8.4V from the Lipo Battery to the 5V raspberry
-  1 x Hexapod Basemodel BH3 from Lynxmotion http://www.lynxmotion.com/c-33-bh3.aspx
-  1 x Simple LiPo low voltage alarm (as a plugg)
-  1 x Adafruit ADS1015 as a LiPo low voltage alarm over I2C

# Software

The following modules should be installed using apt-get after installing Raspbian on the Raspberry Pi:
-  Python 2.7
-  Adafruit Adafruit_PWM_Servo_Driver
-  Adafruit 17006-driver
-  OLED 1306 (courtesy ofRichard Hull)
-  PIL

The basecode for the robot is taken from the Phoenix-project, originally written in Micro Basic for the ARC32 by 
Xan (Jeroen Janssen) 
Zenta (Kåre Halvorsen)
KurtE (Kurt Eckhardt)


Check out my YouTube channel for videos of the first robot in action.
https://www.youtube.com/user/niklasangstuna

Best regards
PappaNiklas


The 1306 code it taken from Richard Hull


Adafruit invests time and resources providing this open source code, please support Adafruit and open-source hardware by purchasing products from Adafruit!

The MIT License (MIT)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

