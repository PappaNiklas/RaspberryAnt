#!/usr/bin/python
# -*- coding: utf-8 -*-


#
# This code is NOT stable !!!
#
# version 0.2
# continous work to get the code errorfree or at least compilable ...
#


print("Loading ptvsd")
import ptvsd
ptvsd.enable_attach(secret = "pi")
a = raw_input("Connect debug pi@192.168.1.244 and press to start")

print("Loading serial,time,datetime, threading, sys, GPIO")
import serial,time,datetime, threading, sys,  RPi.GPIO as GPIO
print("Loading Adafruit_PWM_Servo_Driver")
from Adafruit_PWM_Servo_Driver import PWM #class
print("Loading Adafruit ADS")
from Adafruit_ADS1x15 import ADS1x15
print("Loading threading")
from threading import Thread
print("Loading ConfigParser")
import ConfigParser
print("Loading Queue")
from Queue import Queue
print("Loading OLED")
from oled.device import ssd1306, sh1106
from oled.render import canvas
print("Loading PIL")
#from PIL import Image
from PIL import ImageFont
#from PIL import ImageDraw
print("Loading numpy")
from numpy import interp


debug = True




# ---INIT---
if debug: print("Initializing variables")

buzzer_pin = 23
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(buzzer_pin, GPIO.OUT)



#VAR-init

#[ANGLES]
CoxaAngle1	    = []	#Actual Angle of the horizontal hip, decimals = 1
FemurAngle1		= []	#Actual Angle of the vertical hip, decimals = 1
TibiaAngle1		= []	#Actual Angle of the knee, decimals = 1
for no in range(0,6):   #init
    CoxaAngle1.append(0)
for no in range(0,6):
    FemurAngle1.append(0)
for no in range(0,6):
    TibiaAngle1.append(0)

#[HServo]
aswCoxaServo	= []	# pre-calculated HSERVO values from Angles above
aswFemurServo	= []
aswTibiaServo	= []
for no in range(0,6):   #init
    aswCoxaServo.append(0)
for no in range(0,6):   #init
    aswFemurServo.append(0)
for no in range(0,6):   #init
    aswTibiaServo.append(0)
fFirstMove		= 0		# need to handle first move specially...

#bCSIn		    = 0		# used in Read/Write servo offsets - checksum read
#bCSCalc		= 0		# Used in Read/Write Servo offsets - calculated checksum

#[POSITIONS SINGLE LEG CONTROL]
SLHold	        = 0.0		 	#Single leg control mode
LegPosX	        = [0,0,0,0,0,0]	#Actual X Posion of the Leg
LegPosY	        = [0,0,0,0,0,0]	#Actual Y Posion of the Leg
LegPosZ	        = [0,0,0,0,0,0]	#Actual Z Posion of the Leg

#[INPUTS]
#butA 	var bit
#butB 	var bit
#butC 	var bit

#prev_butA var bit
#prev_butB var bit
#prev_butC var bit

#[OUTPUTS]
#LedA var bit	#Red
#LedB var bit	#Green
#LedC var bit	#Orange
#Eyes var bit	#Eyes output

#[VARIABLES]
Index 			= 0		#Index universal used
LegIndex		= 0		#Index used for leg Index Number
i				= 0		#Index universal used
b				= 0		#Index universal used 

#GetSinCos / ArcCos
AngleDeg1 		= 0.0		#Input Angle in degrees, decimals = 1
ABSAngleDeg1 	= 0.0		#Absolute value of the Angle in Degrees, decimals = 1
sin4         	= 0.0		#Output Sinus of the given Angle, decimals = 4
cos4			= 0.0		#Output Cosinus of the given Angle, decimals = 4
AngleRad4		= 0.0		#Output Angle in radials, decimals = 4
NegativeValue	= 0.0			#If the the value is Negative

#GetATan2
AtanX			= 0.0		#Input X
AtanY			= 0.0		#Input Y
Atan4			= 0.0		#ArcTan2 output
XYHyp2			= 0.0		#Output presenting Hypotenuse of X and Y

#Body position
BodyPosX 		= 0		#Global Input for the position of the body
BodyPosY 		= 0.0
BodyPosZ 		= 0.0

#Body Inverse Kinematics
BodyRotX1		= 0.0   #Global Input pitch of the body
BodyRotY1		= 0.0   #Global Input rotation of the body
BodyRotZ1  		= 0.0   #Global Input roll of the body
PosX			= 0.0   #Input position of the feet X
PosZ			= 0.0   #Input position of the feet Z
PosY			= 0.0   #Input position of the feet Y
RotationY		= 0     #Input for rotation of a single feet for the gait
sinA4          	= 0.0   #Sin buffer for BodyRotX calculations
cosA4          	= 0.0   #Cos buffer for BodyRotX calculations
sinB4          	= 0.0   #Sin buffer for BodyRotX calculations
cosB4          	= 0.0   #Cos buffer for BodyRotX calculations
sinG4          	= 0.0   #Sin buffer for BodyRotZ calculations
cosG4          	= 0.0   #Cos buffer for BodyRotZ calculations
TotalX			= 0.0   #Total X distance between the center of the body and the feet
TotalZ			= 0.0   #Total Z distance between the center of the body and the feet
BodyIKPosX		= 0.0   #Output Position X of feet with Rotation
BodyIKPosY		= 0.0   #Output Position Y of feet with Rotation
BodyIKPosZ		= 0.0   #Output Position Z of feet with Rotation

#Leg Inverse Kinematics
IKFeetPosX	    = 0.0   #Input position of the Feet X
IKFeetPosY	    = 0.0   #Input position of the Feet Y
IKFeetPosZ		= 0.0	#Input Position of the Feet Z
IKFeetPosXZ		= 0.0   #Diagonal direction from Input X and Z
IKSW2			= 0.0	#Length between Shoulder and Wrist, decimals = 2
IKA14		    = 0.0	#Angle of the line S>W with respect to the ground in radians, decimals = 4
IKA24		    = 0.0	#Angle of the line S>W with respect to the femur in radians, decimals = 4
Temp1			= 0
Temp2			= 0
IKSolution		    = 0	#Output true if the solution is possible
IKSolutionWarning 	= 0	#Output true if the solution is NEARLY possible
IKSolutionError		= 0	#Output true if the solution is NOT possible
#--------------------------------------------------------------------
#[TIMING]
lTimerCnt		= 0.0	# used now also in timing of how long since we received a message
lCurrentTime	= 0.0
lTimerStart		= 0.0   #Start time of the calculation cycles
lTimerEnd		= 0.0 	#End time of the calculation cycles
CycleTime		= 0.0	#Total Cycle time
SSCTime  		= 0.0	#Time for servo updates

PrevSSCTime		= 0.0	#Previous time for the servo updates
#ifdef DEBUG_ROT
xxxLastBodyRotZ1= 0.0
fDebugRotDisp	= 0
#endif

InputTimeDelay	= 0.0	#Delay that depends on the input to get the "sneaking" effect
SpeedControl	= 0.0	#Adjustible Delay

#--------------------------------------------------------------------
#[GLOABAL]
HexOn	 	    = 0		#Switch to turn on Phoenix
Prev_HexOn	    = 0	    #Previous loop state 
#--------------------------------------------------------------------
#[Balance]
BalanceMode	    = 0.0
TotalTransX	    = 0.0
TotalTransZ		= 0.0
TotalTransY		= 0.0
TotalYBal1		= 0
TotalXBal1		= 0
TotalZBal1		= 0
TotalY			= 0   #Total Y distance between the center of the body and the feet

#[Single Leg Control]
SelectedLeg		= 0
Prev_SelectedLeg= 0
SLLegX			= 0.0
SLLegY			= 0.0
SLLegZ			= 0.0
AllDown			= 0

#[gait]
GaitType		= 0     #Gait type
NomGaitSpeed	= 0     #Nominal speed of the gait
LegLiftHeight 	= 0.0	#Current Travel height
TravelLengthX 	= 0.0	#Current Travel length X
TravelLengthZ 	= 0.0	#Current Travel length Z
TravelRotationY = 0.0	#Current Travel Rotation Y
TLDivFactor		= 0 	#Number of steps that a leg is on the floor while walking
NrLiftedPos   	= 0 	#Number of positions that a single leg is lifted (1-3)
HalfLiftHeigth	= 0		#If TRUE the outer positions of the ligted legs will be half height	
GaitInMotion 	= 0		#Temp to check if the gait is in motion
StepsInGait		= 0 	#Number of steps in gait
LastLeg 		= 0 	#TRUE when the current leg is the last leg of the sequence
GaitStep 	 	= 0	    #Actual Gait step
GaitLegNr		= []	#Init position of the leg
for no in range(0,6):
    GaitLegNr.append(0)
GaitLegNrIn	 	= 0 	#Input Number of the leg
GaitPosX 		= [0,0,0,0,0,0]    #Array containing Relative X position corresponding to the Gait
GaitPosY 		= [0,0,0,0,0,0]    #Array containing Relative Y position corresponding to the Gait
GaitPosZ 		= [0,0,0,0,0,0]    #Array containing Relative Z position corresponding to the Gait
GaitRotY 		= [0,0,0,0,0,0]    #Array containing Relative Y rotation corresponding to the Gait
GaitPeak      	= 0     # Saving the largest (ABS) peak value from GaitPosX,Y,Z and GaitRotY
Walking         = 0     # True if the robot are walking

BodyAngle   	= 0.0        
TravelLengthXZ	= 0.0
Headtilt		= 0.0



#[CONSTANTS]
BUTTON_DOWN = 0
BUTTON_UP 	= 1

c1DEC		= 10
c2DEC		= 100
c4DEC		= 10000
c6DEC		= 1000000

#Leg numbering
cRR			= 0
cRM			= 1
cRF			= 2
cLR			= 3
cLM			= 4
cLF			= 5


#[PIN NUMBERS]
cRRCoxaPin 		= 0	#Rear Right leg Hip Horizontal
cRRFemurPin 	= 1	#Rear Right leg Hip Vertical
cRRTibiaPin 	= 2	#Rear Right leg Knee

cRMCoxaPin 		= 4	#Middle Right leg Hip Horizontal
cRMFemurPin 	= 5	#Middle Right leg Hip Vertical
cRMTibiaPin 	= 6	#Middle Right leg Knee

cRFCoxaPin 		= 8	#Front Right leg Hip Horizontal
cRFFemurPin 	= 9	#Front Right leg Hip Vertical
cRFTibiaPin 	= 10	#Front Right leg Knee

cLRCoxaPin 		= 16	#Rear Left leg Hip Horizontal
cLRFemurPin 	= 17	#Rear Left leg Hip Vertical
cLRTibiaPin 	= 18	#Rear Left leg Knee

cLMCoxaPin 		= 20	#Middle Left leg Hip Horizontal
cLMFemurPin 	= 21	#Middle Left leg Hip Vertical
cLMTibiaPin 	= 22	#Middle Left leg Knee

cLFCoxaPin 		= 24	#Front Left leg Hip Horizontal
cLFFemurPin 	= 25	#Front Left leg Hip Vertical
cLFTibiaPin 	= 26	#Front Left leg Knee

noServo = 27    #no need to parse servo numbers higher than this

serviDynMa = [
    #snr    fss     minpulse    maxpuls    speed   endpos     
    [0,     0.65,    900,        2100,       0,      1500],     
    [1,     0.65,    900,        2100,       0,      1500],    
    [2,     0.65,    900,        2100,       0,      1500],    
    [3,     0.65,    0,          0,          0,      1500],   
    [4,     0.65,    900,        2100,       0,      1500],    
    [5,     0.65,    900,        2100,       0,      1500],    
    [6,     0.65,    900,        2100,       0,      1500],    
    [7,     0.65,    0,          0,          0,      1500],    
    [8,     0.65,    900,        2100,       0,      1500],    
    [9,     0.65,    900,        2100,       0,      1500],    
    [10,    0.65,    900,        2100,       0,      1500],    
    [11,    0.65,    0,          0,          0,      1500],    
    [12,    0.65,    0,          0,          0,      1500],    
    [13,    0.65,    0,          0,          0,      1500],    
    [14,    0.65,    0,          0,          0,      1500],    
    [15,    0.65,    0,          0,          0,      1500],    
    [16,    0.65,    900,        2100,       0,      1500],    
    [17,    0.65,    900,        2100,       0,      1500],    
    [18,    0.65,    900,        2100,       0,      1500],    
    [19,    0.65,    0,          0,          0,      1500],    
    [20,    0.65,    900,        2100,       0,      1500],    
    [21,    0.65,    900,        2100,       0,      1500],    
    [22,    0.65,    900,        2100,       0,      1500],    
    [23,    0.65,    0,          0,          0,      1500],    
    [24,    0.65,    900,        2100,       0,      1500],    
    [25,    0.65,    900,        2100,       0,      1500],    
    [26,    0.65,    900,        2100,       0,      1500],    
    [27,    0.65,    0,          0,          0,      1500],    
    [28,    0.65,    0,          0,          0,      1500],    
    [29,    0.65,    0,          0,          0,      1500],    
    [30,    0.65,    0,          0,          0,      1500],    
    [31,    0.65,    0,          0,          0,      1500], ]



servoDynTh = [
    #snr    posnow  
    [0,      1500    ],
    [1,      1500    ],
    [2,      1500    ],
    [3,      1500    ],
    [4,      1500    ],
    [5,      1500    ],
    [6,      1500    ],
    [7,      1500    ],
    [8,      1500    ],
    [9,      1500    ],
    [10,      1500    ],
    [11,      1500    ],
    [12,      1500    ],
    [13,      1500    ],
    [14,      1500    ],
    [15,      1500    ],
    [16,      1500    ],
    [17,      1500    ],
    [18,      1500    ],
    [19,      1500    ],
    [20,      1500    ],
    [21,      1500    ],
    [22,      1500    ],
    [23,      1500    ],
    [24,      1500    ],
    [25,      1500    ],
    [26,      1500    ],
    [27,      1500    ],
    [28,      1500    ],
    [29,      1500    ],
    [30,      1500    ],
    [31,      1500    ],
    ]

 
#cHeadRotate	= P20
#cHeadTilt		= P21

#--------------------------------------------------------------------
#[MIN/MAX ANGLES]

cHeadRotateMin1	= -450
cHeadRotateMax1	= 450
cHeadRotateOffset1	= 0
cHeadRotateSteps= 186

cHeadTiltFast1	= 50
cHeadTiltSlow1	= 300
cHeadTiltSteps  = 200


cRFCoxaMin1		= -1150	#Mechanical limits of the Right Front Leg, decimals = 1
cRFCoxaMax1		= 550
cRFFemurMin1	= -800
cRFFemurMax1	= 850
cRFTibiaMin1	= -600
cRFTibiaMax1	= 850

cLFCoxaMin1		= -1150	#Mechanical limits of the Left Front Leg, decimals = 1
cLFCoxaMax1		= 550
cLFFemurMin1	= -800
cLFFemurMax1	= 850
cLFTibiaMin1	= -600
cLFTibiaMax1	= 850


cRMCoxaMin1		= -700	#Mechanical limits of the Right Middle Leg, decimals = 1
cRMCoxaMax1		= 700
cRMFemurMin1	= -800
cRMFemurMax1	= 850
cRMTibiaMin1	= -600
cRMTibiaMax1	= 850

cLMCoxaMin1		= -700	#Mechanical limits of the Left Middle Leg, decimals = 1
cLMCoxaMax1		= 700
cLMFemurMin1	= -800
cLMFemurMax1	= 850
cLMTibiaMin1	= -600
cLMTibiaMax1	= 850


cRRCoxaMin1		= -550	#Mechanical limits of the Right Rear Leg, decimals = 1
cRRCoxaMax1		= 1150
cRRFemurMin1	= -800
cRRFemurMax1	= 850
cRRTibiaMin1	= -600
cRRTibiaMax1	= 850

cLRCoxaMin1		= -550	#Mechanical limits of the Left Rear Leg, decimals = 1
cLRCoxaMax1		= 1150
cLRFemurMin1	= -800
cLRFemurMax1	= 850
cLRTibiaMin1	= -600
cLRTibiaMax1	= 850



#--------------------------------------------------------------------
#[BODY DIMENSIONS]
cCoxaLength  	= 29		#Length of the Coxa [mm]
cFemurLength 	= 75		#Length of the Femur [mm]
cTibiaLength 	= 104		#Lenght of the Tibia [mm]

cRRCoxaAngle1 	= -300	#Default Coxa setup angle, decimals = 1
cRMCoxaAngle1 	= 0		#Default Coxa setup angle, decimals = 1
cRFCoxaAngle1 	= 300		#Default Coxa setup angle, decimals = 1
cLRCoxaAngle1 	= -300	#Default Coxa setup angle, decimals = 1
cLMCoxaAngle1 	= 0		#Default Coxa setup angle, decimals = 1
cLFCoxaAngle1 	= 300		#Default Coxa setup angle, decimals = 1

cRROffsetX 		= -79		#Distance X from center of the body to the Right Rear coxa
cRROffsetZ 		= 152		#Distance Z from center of the body to the Right Rear coxa
cRMOffsetX 		= -79		#158/2=79 Distance X from center of the body to the Right Middle coxa
cRMOffsetZ 		= 0		#Distance Z from center of the body to the Right Middle coxa
cRFOffsetX 		= -79		#Distance X from center of the body to the Right Front coxa
cRFOffsetZ 		= -152	#Distance Z from center of the body to the Right Front coxa

cLROffsetX 		= 79		#Distance X from center of the body to the Left Rear coxa
cLROffsetZ 		= 152		#Distance Z from center of the body to the Left Rear coxa
cLMOffsetX 		= 79		#Distance X from center of the body to the Left Middle coxa
cLMOffsetZ 		= 0		#Distance Z from center of the body to the Left Middle coxa
cLFOffsetX 		= 79		#Distance X from center of the body to the Left Front coxa
cLFOffsetZ 		= -152	#Distance Z from center of the body to the Left Front coxa


#--------------------------------------------------------------------

#[START POSITIONS FEET 
CHexInitY		= 25		#Default heigh

cRRInitPosX 	= (cCoxaLength + cFemurLength) * 0.866		#Start positions of the Right Rear leg (sin30 = 0,5 & cos30 = 0,866)
cRRInitPosY 	= CHexInitY
cRRInitPosZ 	= (cCoxaLength + cFemurLength) * 0.5

cRMInitPosX 	= (cCoxaLength + cFemurLength) 		#Start positions of the Right Middle leg
cRMInitPosY 	= CHexInitY
cRMInitPosZ 	= 0

cRFInitPosX 	= (cCoxaLength + cFemurLength) * 0.866		#Start positions of the Right Front leg
cRFInitPosY 	= CHexInitY
cRFInitPosZ 	= -(cCoxaLength + cFemurLength) * 0.5

cLRInitPosX 	= (cCoxaLength + cFemurLength) * 0.866		#Start positions of the Left Rear leg
cLRInitPosY 	= CHexInitY
cLRInitPosZ 	= (cCoxaLength + cFemurLength) * 0.5

cLMInitPosX 	= (cCoxaLength + cFemurLength)		#Start positions of the Left Middle leg
cLMInitPosY 	= CHexInitY
cLMInitPosZ 	= 0

cLFInitPosX 	= (cCoxaLength + cFemurLength) * 0.866		#Start positions of the Left Front leg
cLFInitPosY 	= CHexInitY
cLFInitPosZ 	= -(cCoxaLength + cFemurLength) * 0.5

#--------------------------------------------------------------------





StepsPerDegree 		= 133 #200 on 645, else 133

print("Initializing Serial")
port = serial.Serial("/dev/ttyAMA0",115200, timeout=1) # need timeout, else the threads hang and it whont terminate is sender is down
port.flushInput()
timeNow = time.time()
ListOKByte = []

print("Initializing Servidrivers")
pwm = PWM(0x40, False)
pwm2 = PWM(0x41, False)
servoFreq = 60
pulseConstant = (1000000/servoFreq/4096)
pwm.setPWMFreq(servoFreq)
pwm2.setPWMFreq(servoFreq)

print("Initializing AD")
adc = ADS1x15(ic=0x00)
gain = 6144  # +/- 6.144V
LiPoReading = 0
LiPoShutOffVoltage = 6000
LiPoLowVoltage = 0          # lets hope for at fresh Lipo at start


#servoMin = 650
#servoMax = 2350
#servoMid = ((servoMax - servoMin) / 2) + servoMin
#byteStep = (servoMax - servoMin)/256

#--------------------------------------------------------------------
#[REMOTE]				 
cTravelDeadZone	= 2	#The deadzone for the analog input from the remote
#--------------------------------------------------------------------

SERVOSAVECNT	=	32				
aServoOffsets	=	[]		# Our new values - must take stored away values into account...
lTimerCnt = 0				# for timekeeping

driverHz = 20
BlockServoUpdates = 0
ServosBeingUpdated = 0


#ArcCosinus Table
#Table build in to 3 part to get higher accuracy near cos = 1. 
#The biggest error is near cos = 1 and has a biggest value of 3*0.012098rad = 0.521 deg.
#-	Cos 0 to 0.9 is done by steps of 0.0079 rad. (1/127)
#-	Cos 0.9 to 0.99 is done by steps of 0.0008 rad (0.1/127)
#-	Cos 0.99 to 1 is done by step of 0.0002 rad (0.01/64)
#Since the tables are overlapping the full range of 127+127+64 is not necessary. Total bytes: 277
GetACos = [255,254,252,251,250,249,247,246,245,243,242,241,240,238,237,236,234,233,232,231,229,228,227,225, 
    224,223,221,220,219,217,216,215,214,212,211,210,208,207,206,204,203,201,200,199,197,196,195,193, 
	192,190,189,188,186,185,183,182,181,179,178,176,175,173,172,170,169,167,166,164,163,161,160,158, 
	157,155,154,152,150,149,147,146,144,142,141,139,137,135,134,132,130,128,127,125,123,121,119,117,
	115,113,111,109,107,105,103,101,98,96,94,92,89,87,84,81,79,76,73,73,73,72,72,72,71,71,71,70,70, 
	70,70,69,69,69,68,68,68,67,67,67,66,66,66,65,65,65,64,64,64,63,63,63,62,62,62,61,61,61,60,60,59, 
	59,59,58,58,58,57,57,57,56,56,55,55,55,54,54,53,53,53,52,52,51,51,51,50,50,49,49,48,48,47,47,47, 
	46,46,45,45,44,44,43,43,42,42,41,41,40,40,39,39,38,37,37,36,36,35,34,34,33,33,32,31,31,30,29,28, 
	28,27,26,25,24,23,23,23,23,22,22,22,22,21,21,21,21,20,20,20,19,19,19,19,18,18,18,17,17,17,17,16, 
	16,16,15,15,15,14,14,13,13,13,12,12,11,11,10,10,9,9,8,7,6,6,5,3,0]

#Sin table 90 deg, persision 0.5 deg (180 values)
GetSin = [0, 87, 174, 261, 348, 436, 523, 610, 697, 784, 871, 958, 1045, 1132, 1218, 1305, 1391, 1478, 1564, 
	1650, 1736, 1822, 1908, 1993, 2079, 2164, 2249, 2334, 2419, 2503, 2588, 2672, 2756, 2840, 2923, 3007, 
	3090, 3173, 3255, 3338, 3420, 3502, 3583, 3665, 3746, 3826, 3907, 3987, 4067, 4146, 4226, 4305, 4383, 
	4461, 4539, 4617, 4694, 4771, 4848, 4924, 4999, 5075, 5150, 5224, 5299, 5372, 5446, 5519, 5591, 5664, 
	5735, 5807, 5877, 5948, 6018, 6087, 6156, 6225, 6293, 6360, 6427, 6494, 6560, 6626, 6691, 6755, 6819, 
	6883, 6946, 7009, 7071, 7132, 7193, 7253, 7313, 7372, 7431, 7489, 7547, 7604, 7660, 7716, 7771, 7826, 
	7880, 7933, 7986, 8038, 8090, 8141, 8191, 8241, 8290, 8338, 8386, 8433, 8480, 8526, 8571, 8616, 8660, 
	8703, 8746, 8788, 8829, 8870, 8910, 8949, 8987, 9025, 9063, 9099, 9135, 9170, 9205, 9238, 9271, 9304, 
	9335, 9366, 9396, 9426, 9455, 9483, 9510, 9537, 9563, 9588, 9612, 9636, 9659, 9681, 9702, 9723, 9743, 
	9762, 9781, 9799, 9816, 9832, 9848, 9862, 9876, 9890, 9902, 9914, 9925, 9935, 9945, 9953, 9961, 9969, 
	9975, 9981, 9986, 9990, 9993, 9996, 9998, 9999, 10000]

#Build tables for Leg configuration like I/O and MIN/MAX values to easy access values using a FOR loop
#Constants are still defined as single values in the cfg file to make it easy to read/configure
#Pin numbers...
cCoxaPin = 	[cRRCoxaPin,  cRMCoxaPin,  cRFCoxaPin,  cLRCoxaPin,  cLMCoxaPin,  cLFCoxaPin]
cFemurPin =	[cRRFemurPin, cRMFemurPin, cRFFemurPin, cLRFemurPin, cLMFemurPin, cLFFemurPin]
cTibiaPin =	[cRRTibiaPin, cRMTibiaPin, cRFTibiaPin, cLRTibiaPin, cLMTibiaPin, cLFTibiaPin]

#Min / Max values
cCoxaMin1 =	[cRRCoxaMin1,  cRMCoxaMin1,  cRFCoxaMin1,  cLRCoxaMin1,  cLMCoxaMin1,  cLFCoxaMin1]
cCoxaMax1 =	[cRRCoxaMax1,  cRMCoxaMax1,  cRFCoxaMax1,  cLRCoxaMax1,  cLMCoxaMax1,  cLFCoxaMax1]
cFemurMin1=	[cRRFemurMin1, cRMFemurMin1, cRFFemurMin1, cLRFemurMin1, cLMFemurMin1, cLFFemurMin1]
cFemurMax1=	[cRRFemurMax1, cRMFemurMax1, cRFFemurMax1, cLRFemurMax1, cLMFemurMax1, cLFFemurMax1]
cTibiaMin1=	[cRRTibiaMin1, cRMTibiaMin1, cRFTibiaMin1, cLRTibiaMin1, cLMTibiaMin1, cLFTibiaMin1]
cTibiaMax1=	[cRRTibiaMax1, cRMTibiaMax1, cRFTibiaMax1, cLRTibiaMax1, cLMTibiaMax1, cLFTibiaMax1]

#Body Offsets (distance between the center of the body and the center of the coxa)
cOffsetX =	[cRROffsetX, cRMOffsetX, cRFOffsetX, cLROffsetX, cLMOffsetX, cLFOffsetX]
cOffsetZ =	[cRROffsetZ, cRMOffsetZ, cRFOffsetZ, cLROffsetZ, cLMOffsetZ, cLFOffsetZ]

#Default leg angle
cCoxaAngle1=[cRRCoxaAngle1, cRMCoxaAngle1, cRFCoxaAngle1, cLRCoxaAngle1, cLMCoxaAngle1, cLFCoxaAngle1]

#Start positions for the leg
cInitPosX =	[cRRInitPosX, cRMInitPosX, cRFInitPosX, cLRInitPosX, cLMInitPosX, cLFInitPosX]
cInitPosY =	[cRRInitPosY, cRMInitPosY, cRFInitPosY, cLRInitPosY, cLMInitPosY, cLFInitPosY]
cInitPosZ =	[cRRInitPosZ, cRMInitPosZ, cRFInitPosZ, cLRInitPosZ, cLMInitPosZ, cLFInitPosZ]

#[Debug Level - ]
#wDebugLevel		var	word	# this is the current debug level, can set by terminal monitor
# note each of the bits can be used any which way Also obviously 
DBG_LVL_NORMAL	= 	0x01	# - Normal starting debug level
DBG_LVL_VERBOSE	=  	0x02	# - More verbose debug
DBG_LVL_CONTROL	=		0x04	# - Turn on debug outputs for the control (PS2,XBEE...)
DBG_LVL_HSERVO	=		0x10	# - Debug HSERVO output...
DBG_LVL_ENTERLEAVE = 	0x80	# Enter/Leave

# Feedback messages 
_PhoenixDebug	= "Starting Phoenix..."
_WALKMODE		= "Walking"
_TANSLATEMODE	= "Body Translate"
_ROTATEMODE		= "Body Rotate"
_SINGLELEG		= "Single Leg"
_BALANCEON		= "Balance On"
_BALANCEOFF		= "Balance Off"


#[CONSTANTS Controller]
WalkMode			= 0
TranslateMode		= 1
RotateMode			= 2
SingleLegMode		= 3
#[Ps2 Controller Variables]
#DualShock 			var Byte(7)
#LastButton 			= 0
DS2Mode 			= 0
PS2Index			= 0
BodyYOffset 		= 0.0
BodyYShift			= 0.0
ControlMode			= 0
DoubleHeightOn		= 0
DoubleTravelOn		= 0
WalkMethod			= 0













# initialize to say that we have not done any moves yet...
fFirstMove = 1						

#Single leg control. Make sure no leg is selected
SelectedLeg = 255 # No Leg selected
Prev_SelectedLeg = 255

#Body Positions
BodyPosX = 0
BodyPosY = 0
BodyPosZ = 0

#Body Rotations
BodyRotX1 = 0
BodyRotY1 = 0
BodyRotZ1 = 0

#Gait
GaitType = 0
BalanceMode = 0
LegLiftHeight = 50
GaitStep = 1

#Tars Init Positions
for LegIndex in range(0, 6):
    LegPosX[LegIndex] = cInitPosX[LegIndex]	#Set start positions for each leg
    LegPosY[LegIndex] = cInitPosY[LegIndex]
    LegPosZ[LegIndex] = cInitPosZ[LegIndex]  

#Move the head to init position
#HSERVO [cHeadRotate\0,cHeadTilt\100]

#Initialize Controller
ListInByteOK = []
LastListInByte = [0,0,0,0,0,0,0]
#InitController()

#SSC
SSCTime = 150

GaitCurrentLegNr = 0

#HexOn = 0 allready done above

#Turning off all the leds
#LedA = 0
#LedB = 0
#LedC = 0
#Eyes = 0



#Function declaration:
if debug: print("Starting function declaration")

#--------------------------------------------------------------------
#[GETSINCOS] Get the sinus and cosinus from the angle +/- multiple circles
#AngleDeg1 	- Input Angle in degrees
#Sin4    	- Output Sinus of AngleDeg
#Cos4  		- Output Cosinus of AngleDeg
def GetSinCos(AngleDeg1):
    global Sin4
    global Cos4
    if debug: print("GetSinCon Input ", AngleDeg1)
    #Get the absolute value of AngleDeg
    if AngleDeg1 < 0 :  
        ABSAngleDeg1 = AngleDeg1 *-1
    else:               
        ABSAngleDeg1 = AngleDeg1
    #Shift rotation to a full circle of 360 deg -> AngleDeg // 360
    if AngleDeg1 < 0:	#Negative values
        AngleDeg1 = 3600-(ABSAngleDeg1-(3600*(ABSAngleDeg1/3600)))
    else:				#Positive values
        AngleDeg1 = ABSAngleDeg1-(3600*(ABSAngleDeg1/3600))
    if (AngleDeg1>=0 and AngleDeg1<=900):	# 0 to 90 deg
        Sin4 = GetSin[int(round(AngleDeg1/5))] 			# 5 is the presision (0.5) of the table
        Cos4 = GetSin[int(round((900-(AngleDeg1))/5))] 	
    elif (AngleDeg1>900 and AngleDeg1<=1800): 	# 90 to 180 deg
        Sin4 = GetSin[int(round((900-(AngleDeg1-900))/5))] # 5 is the presision (0.5) of the table	
        Cos4 = -GetSin[int(round((AngleDeg1-900)/5))]			
    elif (AngleDeg1>1800 and AngleDeg1<=2700): # 180 to 270 deg
        Sin4 = -GetSin[int(round((AngleDeg1-1800)/5))] 	# 5 is the presision (0.5) of the table
        Cos4 = -GetSin[int(round((2700-AngleDeg1)/5))]
    elif (AngleDeg1>2700 and AngleDeg1<=3600): # 270 to 360 deg
        Sin4 = -GetSin[int(round((3600-AngleDeg1)/5))] # 5 is the presision (0.5) of the table	
        Cos4 = GetSin[int(round((AngleDeg1-2700)/5))]	
    if debug: print("GetSinCon output Sin4 ", Sin4)		
    if debug: print("GetSinCon output Cos4 ", Cos4)		



#--------------------------------------------------------------------
#[GETARCCOS] Get the sinus and cosinus from the angle +/- multiple circles
#Cos4    	- Input Cosinus
#AngleRad4 	- Output Angle in AngleRad4
def GetArcCos(Cos4):
    if debug: print("GetArcCos Input ", Cos4)
    #Check for negative value
    if (Cos4<0):
        Cos4 = -Cos4
        NegativeValue = 1
    else:
        NegativeValue = 0
    #Limit Cos4 to his maximal value
    if Cos4> c4DEC:
        Cos4 = c4DEC 
    if (Cos4>=0 and Cos4<9000):
        AngleRad4 = GetACos(Cos4/79) #79=table resolution (1/127)
        AngleRad4 = AngleRad4*616/c1DEC #616=acos resolution (pi/2/255) 
    elif (Cos4>=9000 and Cos4<9900):
        AngleRad4 = GetACos((Cos4-9000)/8+114) #8=table resolution (0.1/127), 114 start address 2nd bytetable range 
        AngleRad4 = AngleRad4*616/c1DEC #616=acos resolution (pi/2/255) 
    elif (Cos4>=9900 and Cos4<=10000):
        AngleRad4 = GetACos((Cos4-9900)/2+227) #2=table resolution (0.01/64), 227 start address 3rd bytetable range 
        AngleRad4 = AngleRad4*616/c1DEC #616=acos resolution (pi/2/255) 
    #Add negative sign
    if NegativeValue:
        AngleRad4 = 31416 - AngleRad4
    if debug: print("GetArcCos output AngleRad4 ", AngleRad4)		
    return AngleRad4


#--------------------------------------------------------------------
#[GetATan2] Simplyfied ArcTan2 function based on fixed point ArcCos
#ATanX 		- Input X
#ATanY 		- Input Y
#ATan4  		- Output ARCTAN2(X/Y)
#XYHyp2			- Output presenting Hypotenuse of X and Y
def GetATan2(AtanX, AtanY):
    global XYHyp2
    XYHyp2 = ((AtanX*AtanX*c4DEC) + (AtanY*AtanY*c4DEC))**2 #sqr
    GetArcCos(AtanX*c6DEC / XYHyp2)
    Atan4 = AngleRad4 * (AtanY/abs(AtanY)) #Add sign 
    if debug: print("GetATan2 output Atan4 ", Atan4)
    return Atan4



#--------------------------------------------------------------------
#[BalanceBody]
def BalanceBody():
    global TotalTransY
    global TotalTransZ
    global TotalTransX
    global TotalYBal1
    global TotalZBal1 
    global TotalXBal1
    if debug: print("Balancing body")
    TotalTransZ = TotalTransZ/6 
    TotalTransX = TotalTransX/6
    TotalTransY = TotalTransY/6
    if TotalYBal1 > 0:		#Rotate balance circle by +/- 180 deg
        TotalYBal1 = TotalYBal1 - 1800
    else:
        TotalYBal1 = TotalYBal1 + 1800	
    if TotalZBal1 < -1800: 	#Compensate for extreme balance positions that causes owerflow
        TotalZBal1 = TotalZBal1 + 3600
    if TotalXBal1 < -1800:	#Compensate for extreme balance positions that causes owerflow
        TotalXBal1 = TotalXBal1 + 3600
    #Balance rotation
    TotalYBal1 = -TotalYBal1/6
    TotalXBal1 = -TotalXBal1/6
    TotalZBal1 = TotalZBal1/6

#[BalCalcOneLeg]
def BalCalcOneLeg(PosX, PosZ, PosY, BalLegNr):
    global TotalTransY
    global TotalTransZ
    global TotalTransX
    global TotalYBal1
    global TotalZBal1 
    global TotalXBal1
    if debug: print("Balancing leg ", BalLegNr)
    #Calculating totals from center of the body to the feet
    TotalZ = cOffsetZ[BalLegNr]+PosZ
    TotalX = cOffsetX[BalLegNr]+PosX
    TotalY = 150 + PosY                 #using the value 150 to lower the centerpoint of rotation 'BodyPosY +
    TotalTransY = TotalTransY + PosY
    TotalTransZ = TotalTransZ + TotalZ
    TotalTransX = TotalTransX + TotalX
    GetATan2(TotalX, TotalZ)
    TotalYBal1 =  TotalYBal1 + (ATan4*1800) / 31415
    GetATan2(TotalX, TotalY)
    TotalZBal1 = TotalZBal1 + ((ATan4*1800) / 31415) -900 #Rotate balance circle 90 deg
    GetATan2(TotalZ, TotalY)
    TotalXBal1 = TotalXBal1 + ((ATan4*1800) / 31415) - 900 #Rotate balance circle 90 deg




#--------------------------------------------------------------------
#[BODY INVERSE KINEMATICS] 
#BodyRotX         - Global Input pitch of the body 
#BodyRotY         - Global Input rotation of the body 
#BodyRotZ         - Global Input roll of the body 
#RotationY         - Input Rotation for the gait 
#PosX            - Input position of the feet X 
#PosZ            - Input position of the feet Z 
#SinB          		- Sin buffer for BodyRotX
#CosB           	- Cos buffer for BodyRotX
#SinG          		- Sin buffer for BodyRotZ
#CosG           	- Cos buffer for BodyRotZ
#BodyIKPosX         - Output Position X of feet with Rotation 
#BodyIKPosY         - Output Position Y of feet with Rotation 
#BodyIKPosZ         - Output Position Z of feet with Rotation
#xxxLastBodyRotZ1	var	sword

#BodyIKLeg var nib
def BodyIK(PosX, PosZ, PosY, RotationY, BodyIKLeg) :
    global BodyIKPosX
    global BodyIKPosY
    global BodyIKPosZ
    if debug: print("Doing BodyIK")
    #Calculating totals from center of the body to the feet 
    TotalZ = cOffsetZ[BodyIKLeg]+PosZ 
    TotalX = cOffsetX[BodyIKLeg]+PosX 
    #PosY are equal to a "TotalY" 
  
    #Successive global rotation matrix: 
    #Math shorts for rotation: Alfa (A) = Xrotate, Beta (B) = Zrotate, Gamma (G) = Yrotate 
    #Sinus Alfa = sinA, cosinus Alfa = cosA. and so on... 
  
    #First calculate sinus and cosinus for each rotation: 
    GetSinCos(BodyRotX1+TotalXBal1)
    SinG4 = Sin4
    CosG4 = Cos4
  
    GetSinCos(BodyRotZ1+TotalZBal1)
    SinB4 = Sin4
    CosB4 = Cos4
  
    GetSinCos(BodyRotY1+(RotationY*c1DEC)+TotalYBal1)
    SinA4 = Sin4
    CosA4 = Cos4

    #Calcualtion of rotation matrix: 
    #BodyIKPosX = TotalX - (TotalX*CosA*CosB - TotalZ*CosB*SinA + PosY*SinB)  
    #BodyIKPosZ = TotalZ - (TotalX*CosG*SinA + TotalX*CosA*SinB*SinG + TotalZ*CosA*CosG - TotalZ*SinA*SinB*SinG - PosY*CosB*SinG)   
    #BodyIKPosY = PosY   - (TotalX*SinA*SinG - TotalX*CosA*CosG*SinB + TotalZ*CosA*SinG + TotalZ*CosG*SinA*SinB + PosY*CosB*CosG) 
    BodyIKPosX = (TotalX*c2DEC - ( TotalX*c2DEC*CosA4/c4DEC*CosB4/c4DEC - TotalZ*c2DEC*CosB4/c4DEC*SinA4/c4DEC + PosY*c2DEC*SinB4/c4DEC ))/c2DEC
    BodyIKPosZ = (TotalZ*c2DEC - ( TotalX*c2DEC*CosG4/c4DEC*SinA4/c4DEC + TotalX*c2DEC*CosA4/c4DEC*SinB4/c4DEC*SinG4/c4DEC + TotalZ*c2DEC*CosA4/c4DEC*CosG4/c4DEC - TotalZ*c2DEC*SinA4/c4DEC*SinB4/c4DEC*SinG4/c4DEC - PosY*c2DEC*CosB4/c4DEC*SinG4/c4DEC ))/c2DEC
    BodyIKPosY = (PosY  *c2DEC - ( TotalX*c2DEC*SinA4/c4DEC*SinG4/c4DEC - TotalX*c2DEC*CosA4/c4DEC*CosG4/c4DEC*SinB4/c4DEC + TotalZ*c2DEC*CosA4/c4DEC*SinG4/c4DEC + TotalZ*c2DEC*CosG4/c4DEC*SinA4/c4DEC*SinB4/c4DEC + PosY*c2DEC*CosB4/c4DEC*CosG4/c4DEC ))/c2DEC
  
    #if fDebugRotDisp:
    #    print("B IK ", sdec BodyRotZ1, " ", sdec TotalZBal1, " ", sdec PosX, " ", sdec PosZ, " ", sdec posY, " ", 
    #			sdec RotationY, " ", sdec bodyIKLeg, " ", 
    #			sdec bodyIKPosX, " ", sdec bodyikposz, " ", sdec bodyikposy)




#--------------------------------------------------------------------
#[LEG INVERSE KINEMATICS] Calculates the angles of the coxa, femur and tibia for the given position of the feet
#IKFeetPosX			- Input position of the Feet X
#IKFeetPosY			- Input position of the Feet Y
#IKFeetPosZ			- Input Position of the Feet Z
#IKSolution			- Output true IF the solution is possible
#IKSolutionWarning 	- Output true IF the solution is NEARLY possible
#IKSolutionError	- Output true IF the solution is NOT possible
#FemurAngle1	   	- Output Angle of Femur in degrees
#TibiaAngle1  	 	- Output Angle of Tibia in degrees
#CoxaAngle1			- Output Angle of Coxa in degrees

#LegIKLegNr var nib
def LegIK(IKFeetPosX, IKFeetPosY, IKFeetPosZ, LegIKLegNr):
    global IKSolution
    global IKSolutionWarning
    global IKSolutionError
    global FemurAngle1
    global TibiaAngle1
    global CoxaAngle1
    if debug: print("Doing LegIK on leg ", LegIKLegNr)
    #Calculate IKCoxaAngle and IKFeetPosXZ
    GetATan2(IKFeetPosX, IKFeetPosZ)
    CoxaAngle1[LegIKLegNr] = ((ATan4*180) / 3141) + cCoxaAngle1[LegIKLegNr]
    #Length between the Coxa and tars (foot)
    IKFeetPosXZ = XYHyp2/c2DEC
    #Using GetATan2 for solving IKA1 and IKSW
    #IKA14 - Angle between SW line and the ground in radians
    IKA14 = GetATan2(IKFeetPosY, IKFeetPosXZ-cCoxaLength) 
    #IKSW2 - Length between femur axis and tars
    IKSW2 = XYHyp2
    #if fDebugRotDisp:
    #	print(" == L IK ", sdec IKFeetPosY, " ",sdec IKFeetPosXZ-cCoxaLength, " ", 
    #			sdec IKA14, " ", sdec XYHyp2)
    #IKA2 - Angle of the line S>W with respect to the femur in radians
    Temp1 = (((cFemurLength*cFemurLength) - (cTibiaLength*cTibiaLength))*c4DEC + (IKSW2*IKSW2))
    Temp2 = ((2*cFemurlength)*c2DEC * IKSW2)
    IKA24 = GetArcCos(Temp1 / (Temp2/c4DEC)) 	
    #if fDebugRotDisp:
    #	print("! ", sdec Temp1, " ",sdec Temp1 / (Temp2/c4DEC), " ", 
    #			sdec IKA24, "!")
    #IKFemurAngle
    FemurAngle1[LegIKLegNr] = -(IKA14 + IKA24) * 180 / 3141 + 900

    #IKTibiaAngle
    Temp1 = (((cFemurLength*cFemurLength) + (cTibiaLength*cTibiaLength))*c4DEC - (IKSW2*IKSW2))
    Temp2 = (2*cFemurlength*cTibiaLength)
    GetArcCos(Temp1 / Temp2)
    TibiaAngle1[LegIKLegNr] = -(900-AngleRad4*180/3141)
    #Set the Solution quality	
    if (IKSW2 < (cFemurLength+cTibiaLength-30)*c2DEC):
        IKSolution = 1
        if debug: print("IK Solution is possible")
    else:
        if (IKSW2 < (cFemurLength+cTibiaLength)*c2DEC):
            IKSolutionWarning = 1
            if debug: print("IK Solution is ALLMOST possible")
        else:
            IKSolutionError = 1	
            if debug: print("IK Solution is NOT possible")
    #if fDebugRotDisp then
    #	print(" == L IK ", sdec CoxaAngle1(LegIKLegNr), " ", 
    #			sdec FemurAngle1(LegIKLegNr), " ", sdec TibiaAngle1(LegIKLegNr), 
    #			" SWE: ", hex IKSolution, hex IKSolutionWarning, hex IKSolutionError, 13)


#--------------------------------------------------------------------



def Gait(GaitCurrentLegNr) :
    global GaitInMotion
    global TravelLengthX
    global TravelLengthZ
    global TravelRotationY
    global GaitPosX
    global GaitPosY
    global GaitPosZ
    global GaitRotY
    global GaitStep
    if debug: print("Gait for leg no ",GaitCurrentLegNr)
    #Check IF the Gait is in motion
    GaitInMotion = ((abs(TravelLengthX)>cTravelDeadZone) or (abs(TravelLengthZ)>cTravelDeadZone) or (abs(TravelRotationY)>cTravelDeadZone) )
    if debug: print("TravelLengthX",TravelLengthX)
    if debug: print("TravelLengthZ",TravelLengthZ)
    if debug: print("TravelRotationY",TravelRotationY)
    #Clear values under the cTravelDeadZone
    if (GaitInMotion==0) :
        if debug: print("Value is under the cTravelDeadZone")
        TravelLengthX=0
        TravelLengthZ=0
        TravelRotationY=0
    #Leg middle up position
    #Gait in motion                                                                                     Gait NOT in motion, return to home position
    if (GaitInMotion and (NrLiftedPos==1 or NrLiftedPos==3) or GaitStep==GaitLegNr[GaitCurrentLegNr]) or (not GaitInMotion and GaitStep==GaitLegNr[GaitCurrentLegNr] and ((abs(GaitPosX[GaitCurrentLegNr])>2) or (abs(GaitPosZ[GaitCurrentLegNr])>2) or (abs(GaitRotY[GaitCurrentLegNr])>2))) :	#Up
        if debug: print("Leg middle up position")
        GaitPosX[GaitCurrentLegNr] = 0
        GaitPosY[GaitCurrentLegNr] = -LegLiftHeight
        GaitPosZ[GaitCurrentLegNr] = 0
        GaitRotY[GaitCurrentLegNr] = 0
    else:
        #Optional Half heigth Rear
        if ((NrLiftedPos==2 and GaitStep==GaitLegNr[GaitCurrentLegNr]) or (NrLiftedPos==3 and (GaitStep==GaitLegNr[GaitCurrentLegNr]-1 or GaitStep==GaitLegNr[GaitCurrentLegNr]+(StepsInGait-1)))) and GaitInMotion :
            if debug: print("Optional Half heigth Rear")
            GaitPosX[GaitCurrentLegNr] = -TravelLengthX/2
            GaitPosY[GaitCurrentLegNr] = -LegLiftHeight/(HalfLiftHeigth+1)
            GaitPosZ[GaitCurrentLegNr] = -TravelLengthZ/2
            GaitRotY[GaitCurrentLegNr] = -TravelRotationY/2
        else:
            #Optional half heigth front
            if (NrLiftedPos>=2) and (GaitStep==GaitLegNr[GaitCurrentLegNr]+1 or GaitStep==GaitLegNr[GaitCurrentLegNr]-(StepsInGait-1)) and GaitInMotion :
                if debug: print("Optional half heigth front")
                GaitPosX[GaitCurrentLegNr] = TravelLengthX/2
                GaitPosY[GaitCurrentLegNr] = -LegLiftHeight/(HalfLiftHeigth+1)
                GaitPosZ[GaitCurrentLegNr] = TravelLengthZ/2
                GaitRotY[GaitCurrentLegNr] = TravelRotationY/2
            else :
                #Leg front down position
                if (GaitStep==GaitLegNr[GaitCurrentLegNr]+NrLiftedPos or GaitStep==GaitLegNr[GaitCurrentLegNr]-(StepsInGait-NrLiftedPos)) and GaitPosY[GaitCurrentLegNr]<0 :
                    if debug: print("Leg front down position")
                    GaitPosX[GaitCurrentLegNr] = TravelLengthX/2
                    GaitPosZ[GaitCurrentLegNr] = TravelLengthZ/2
                    GaitRotY[GaitCurrentLegNr] = TravelRotationY/2      	
                    GaitPosY[GaitCurrentLegNr] = 0	##Only move leg down at once if terrain adaption is turned off, SAY WHAT ????
                #Move body forward      
                else:
                    if debug: print("Move body forward")
                    GaitPosX[GaitCurrentLegNr] = GaitPosX[GaitCurrentLegNr] - (TravelLengthX/TLDivFactor)     
                    GaitPosY[GaitCurrentLegNr] = 0  
                    GaitPosZ[GaitCurrentLegNr] = GaitPosZ[GaitCurrentLegNr] - (TravelLengthZ/TLDivFactor)
                    GaitRotY[GaitCurrentLegNr] = GaitRotY[GaitCurrentLegNr] - (TravelRotationY/TLDivFactor)
    #Advance to the next step
    if LastLeg :    #The last leg in this step
        GaitStep = GaitStep+1
        if GaitStep>StepsInGait :
            GaitStep = 1


#--------------------------------------------------------------------
#[CHECK ANGLES] Checks the mechanical limits of the servos
def CheckAngles():
    global CoxaAngle1
    global FemurAngle1
    global TibiaAngle1
    if debug: 
        print("Checking the mechanical limits of the servos, status before:")
        print(CoxaAngle1)
        print(FemurAngle1)
        print(TibiaAngle1)
    for LegIndex in range(0,6):
        if CoxaAngle1[LegIndex]  < cCoxaMin1[LegIndex]  : CoxaAngle1[LegIndex]  = cCoxaMin1[LegIndex]   # after the default angle is added/substracted
        if CoxaAngle1[LegIndex]  > cCoxaMax1[LegIndex]  : CoxaAngle1[LegIndex]  = cCoxaMax1[LegIndex]
        if FemurAngle1[LegIndex] < cFemurMin1[LegIndex] : FemurAngle1[LegIndex] = cFemurMin1[LegIndex]
        if FemurAngle1[LegIndex] > cFemurMax1[LegIndex] : FemurAngle1[LegIndex] = cFemurMax1[LegIndex]
        if TibiaAngle1[LegIndex] < cTibiaMin1[LegIndex] : TibiaAngle1[LegIndex] = cTibiaMin1[LegIndex]
        if TibiaAngle1[LegIndex] > cTibiaMax1[LegIndex] : TibiaAngle1[LegIndex] = cTibiaMax1[LegIndex]
        #BodyAngle = (BodyAngle * 10 + cHeadRotateOffset1) MIN cHeadRotateMin1 MAX cHeadRotateMax1
        #HeadTilt = HeadTilt MIN cHeadTiltFast1 MAX cHeadTiltSlow1
        #print("BodyAngle:   ", BodyAngle)
        #print("   HeadTilt: ", HeadTilt)
    print("Checking the mechanical limits of the servos, status after:")
    print(CoxaAngle1)
    print(FemurAngle1)
    print(TibiaAngle1)


def ServoDriverStart():
    global aswCoxaServo
    global aswFemurServo
    global swTibiaServo
    if debug: 
        print("Initializing servo matrix for each leg")
    for LegIndex in range(0,3):     #Update Right Legs
        #if debug: print("LegIndex: ",LegIndex)
        #if debug: print("cCoxaPin[LegIndex]: ",cCoxaPin[LegIndex])
        aswCoxaServo[LegIndex] = (-CoxaAngle1[LegIndex] * StepsPerDegree ) / 10  + aServoOffsets[cCoxaPin[LegIndex]]	# Convert angle to HServo value
        aswFemurServo[LegIndex] = (-FemurAngle1[LegIndex] * StepsPerDegree ) / 10 + aServoOffsets[cFemurPin[LegIndex]]
        aswTibiaServo[LegIndex] = (-TibiaAngle1[LegIndex] * StepsPerDegree ) / 10 + aServoOffsets[cTibiaPin[LegIndex]]
    for LegIndex in range(3,6):     # Left Legs
        aswCoxaServo[LegIndex] = (CoxaAngle1[LegIndex] * StepsPerDegree ) / 10 + aServoOffsets[cCoxaPin[LegIndex]]
        aswFemurServo[LegIndex] = (FemurAngle1[LegIndex] * StepsPerDegree ) / 10	+ aServoOffsets[cFemurPin[LegIndex]]
        aswTibiaServo[LegIndex] = (TibiaAngle1[LegIndex] * StepsPerDegree ) / 10 + aServoOffsets[cTibiaPin[LegIndex]]
    if debug: 
        print(aswCoxaServo)
        print(aswFemurServo)
        print(aswTibiaServo)

#--------------------------------------------------------------------
#[Servo Driver Commit] Do the actual HSERVO in the commit phase
def ServoDriverCommit():
#	if fFirstMove :
#		fFirstMove = 0		# Clear it...
#		hservo([cRFCoxaPin,aswCoxaServo(cRF)], 
#				[cRFFemurPin,aswFemurServo(cRF)], 
#				[cRFTibiaPin,aswTibiaServo(cRF)], 
#				[cRMCoxaPin,aswCoxaServo(cRM)], 
#				[cRMFemurPin,aswFemurServo(cRM)], 
#				[cRMTibiaPin,aswTibiaServo(cRM)], 
#	  			[cRRCoxaPin,aswCoxaServo(cRR)], 
#				[cRRFemurPin,aswFemurServo(cRR)], 
#	  			[cRRTibiaPin,aswTibiaServo(cRR)], 
#				[cLFCoxaPin,aswCoxaServo(cLF)], 
#	  			[cLFFemurPin,aswFemurServo(cLF)], 
#				[cLFTibiaPin,aswTibiaServo(cLF)], 
#	  			[cLMCoxaPin,aswCoxaServo(cLM)], 
#				[cLMFemurPin,aswFemurServo(cLM)], 
#	  			[cLMTibiaPin,aswTibiaServo(cLM)], 
#				[cLRCoxaPin,aswCoxaServo(cLR)], 
#	 			[cLRFemurPin,aswFemurServo(cLR)], 
#				[cLRTibiaPin,aswTibiaServo(cLR)])
#	else:                                           
#		hservo([cRFCoxaPin,aswCoxaServo(cRF),abs(HServoPos(cRFCoxaPin) - aswCoxaServo(cRF)) * 20 / SSCTime], 
#				[cRFFemurPin,aswFemurServo(cRF),abs(HServoPos(cRFFemurPin) - aswFemurServo(cRF)) * 20 / SSCTime], 
#				[cRFTibiaPin,aswTibiaServo(cRF),abs(HServoPos(cRFTibiaPin) - aswTibiaServo(cRF)) * 20 / SSCTime], 
#				[cRMCoxaPin,aswCoxaServo(cRM),abs(HServoPos(cRMCoxaPin) - aswCoxaServo(cRM)) * 20 / SSCTime], 
#				[cRMFemurPin,aswFemurServo(cRM),abs(HServoPos(cRMFemurPin) - aswFemurServo(cRM)) * 20 / SSCTime], 
#				[cRMTibiaPin,aswTibiaServo(cRM),abs(HServoPos(cRMTibiaPin) - aswTibiaServo(cRM)) * 20 / SSCTime], 
#	  			[cRRCoxaPin,aswCoxaServo(cRR),abs(HServoPos(cRRCoxaPin) - aswCoxaServo(cRR)) * 20 / SSCTime], 
#				[cRRFemurPin,aswFemurServo(cRR),abs(HServoPos(cRRFemurPin) - aswFemurServo(cRR)) * 20 / SSCTime], 
#	  			[cRRTibiaPin,aswTibiaServo(cRR),abs(HServoPos(cRRTibiaPin) - aswTibiaServo(cRR)) * 20 / SSCTime], 
#				[cLFCoxaPin,aswCoxaServo(cLF),abs(HServoPos(cLFCoxaPin) - aswCoxaServo(cLF)) * 20 / SSCTime], 
#	  			[cLFFemurPin,aswFemurServo(cLF),abs(HServoPos(cLFFemurPin) - aswFemurServo(cLF)) * 20 / SSCTime], 
#				[cLFTibiaPin,aswTibiaServo(cLF),abs(HServoPos(cLFTibiaPin) - aswTibiaServo(cLF)) * 20 / SSCTime], 
#	  			[cLMCoxaPin,aswCoxaServo(cLM),abs(HServoPos(cLMCoxaPin) - aswCoxaServo(cLM)) * 20 / SSCTime], 
#				[cLMFemurPin,aswFemurServo(cLM),abs(HServoPos(cLMFemurPin) - aswFemurServo(cLM)) * 20 / SSCTime], 
#	  			[cLMTibiaPin,aswTibiaServo(cLM),abs(HServoPos(cLMTibiaPin) - aswTibiaServo(cLM)) * 20 / SSCTime], 
#				[cLRCoxaPin,aswCoxaServo(cLR),abs(HServoPos(cLRCoxaPin) - aswCoxaServo(cLR)) * 20 / SSCTime], 
#	 			[cLRFemurPin,aswFemurServo(cLR),abs(HServoPos(cLRFemurPin) - aswFemurServo(cLR)) * 20 / SSCTime], 
#				[cLRTibiaPin,aswTibiaServo(cLR),abs(HServoPos(cLRTibiaPin) - aswTibiaServo(cLR)) * 20 / SSCTime])
    if debug: print("SSCTime before hservosync: ", SSCTime)
    hservosync([[cRFCoxaPin,aswCoxaServo[cRF]], 
				[cRFFemurPin,aswFemurServo[cRF]], 
				[cRFTibiaPin,aswTibiaServo[cRF]], 
				[cRMCoxaPin,aswCoxaServo[cRM]], 
				[cRMFemurPin,aswFemurServo[cRM]], 
				[cRMTibiaPin,aswTibiaServo[cRM]], 
	  			[cRRCoxaPin,aswCoxaServo[cRR]], 
				[cRRFemurPin,aswFemurServo[cRR]], 
	  			[cRRTibiaPin,aswTibiaServo[cRR]], 
				[cLFCoxaPin,aswCoxaServo[cLF]], 
	  			[cLFFemurPin,aswFemurServo[cLF]], 
				[cLFTibiaPin,aswTibiaServo[cLF]], 
	  			[cLMCoxaPin,aswCoxaServo[cLM]], 
				[cLMFemurPin,aswFemurServo[cLM]], 
	  			[cLMTibiaPin,aswTibiaServo[cLM]], 
				[cLRCoxaPin,aswCoxaServo[cLR]], 
	 			[cLRFemurPin,aswFemurServo[cLR]], 
				[cLRTibiaPin,aswTibiaServo[cLR]]],SSCTime)
    #hservo([cHeadRotate,-(BodyAngle*cHeadRotateSteps/10),200],[cHeadTilt,(HeadTilt*cHeadTiltSteps/10)])
    PrevSSCTime = SSCTime


    # Feedback - Gait names *** Warning: make sure counts match as I hop through the strings"
    #_GATENAMES		= ["Ripple 6","Ripple 12","Quadripple 9","Tripod 4","Tripod 6","Tripod 8","Wave 12","Trip Tri 12"]	# 	"Wave 18"


def GaitSelect():
        global Gaitname
        global GaitLegNr
        global NrLiftedPos
        global HalfLiftHeigth
        global TLDivFactor
        global StepsInGait
        global NomGaitSpeed
        if GaitType == 0:  #Ripple Gait 6 steps
            Gaitname = "Ripple 6"
            GaitLegNr[cLR] = 1
            GaitLegNr[cRF] = 2	
            GaitLegNr[cLM] = 3	  
            GaitLegNr[cRR] = 4	  
            GaitLegNr[cLF] = 5	  
            GaitLegNr[cRM] = 6
            NrLiftedPos = 1
            HalfLiftHeigth = 0	
            TLDivFactor = 4
            StepsInGait = 6
            NomGaitSpeed = 100
        if GaitType == 1:  #Ripple Gait 12 steps
            Gaitname = "Ripple 12"
            GaitLegNr[cLR] = 1
            GaitLegNr[cRF] = 3
            GaitLegNr[cLM] = 5
            GaitLegNr[cRR] = 7
            GaitLegNr[cLF] = 9
            GaitLegNr[cRM] = 11
            NrLiftedPos = 3
            HalfLiftHeigth = 1
            TLDivFactor = 8	  
            StepsInGait = 12	
            NomGaitSpeed = 50
        if GaitType == 2: #Quadripple 9 steps
            Gaitname = "Quadripple 9"
            GaitLegNr[cLR] = 1    
            GaitLegNr[cRF] = 2
            GaitLegNr[cLM] = 4	  
            GaitLegNr[cRR] = 5
            GaitLegNr[cLF] = 7
            GaitLegNr[cRM] = 8    
            NrLiftedPos = 2
            HalfLiftHeigth = 0	
            TLDivFactor = 6	  
            StepsInGait = 9	    
            NomGaitSpeed = 100
        if GaitType == 3 : #Tripod 4 steps
            Gaitname = "Tripod 4"
            GaitLegNr[cLR] = 3    
            GaitLegNr[cRF] = 1
            GaitLegNr[cLM] = 1
            GaitLegNr[cRR] = 1
            GaitLegNr[cLF] = 3
            GaitLegNr[cRM] = 3
            NrLiftedPos = 1	
            HalfLiftHeigth = 0		
            TLDivFactor = 2	  
            StepsInGait = 4	    
            NomGaitSpeed = 150
        if GaitType == 4 : #Tripod 6 steps
            Gaitname = "Tripod 6"
            GaitLegNr[cLR] = 4    
            GaitLegNr[cRF] = 1
            GaitLegNr[cLM] = 1
            GaitLegNr[cRR] = 1
            GaitLegNr[cLF] = 4
            GaitLegNr[cRM] = 4
            NrLiftedPos = 2
            HalfLiftHeigth = 0	
            TLDivFactor = 4	  
            StepsInGait = 6	    
            NomGaitSpeed = 100
        if GaitType == 5 : #Tripod 8 steps
            Gaitname = "Tripod 8"
            GaitLegNr[cLR] = 5
            GaitLegNr[cRF] = 1
            GaitLegNr[cLM] = 1
            GaitLegNr[cRR] = 1
            GaitLegNr[cLF] = 5
            GaitLegNr[cRM] = 5
            NrLiftedPos = 3
            HalfLiftHeigth = 1	
            TLDivFactor = 4	  
            StepsInGait = 8	    
            NomGaitSpeed = 50
        if GaitType == 6 : #Wave 12 steps
            Gaitname = "Wave 12"
            GaitLegNr[cLR] = 1
            GaitLegNr[cRF] = 11
            GaitLegNr[cLM] = 3
            GaitLegNr[cRR] = 7
            GaitLegNr[cLF] = 5
            GaitLegNr[cRM] = 9
            NrLiftedPos = 1
            HalfLiftHeigth = 0	
            TLDivFactor = 10	  
            StepsInGait = 12	    
            NomGaitSpeed = 85
        if GaitType == 7 : #Triple Tripod 12 steps
            Gaitname = "TripTri 12"
            GaitLegNr[cRF] = 3
            GaitLegNr[cLM] = 4
            GaitLegNr[cRR] = 5
            GaitLegNr[cLF] = 9
            GaitLegNr[cRM] = 10
            GaitLegNr[cLR] = 11
            NrLiftedPos = 3
            HalfLiftHeigth = 3 	
            TLDivFactor = 8  
            StepsInGait = 12	    
            NomGaitSpeed = 60
        if GaitType == 7 : #Wave 18 steps
            Gaitname = "Wave 18"
            GaitLegNr[cLR] = 4 
            GaitLegNr[cRF] = 1
            GaitLegNr[cLM] = 7
            GaitLegNr[cRR] = 13
            GaitLegNr[cLF] = 10
            GaitLegNr[cRM] = 16
            NrLiftedPos = 2
            HalfLiftHeigth = 0	
            TLDivFactor = 16	  
            StepsInGait = 18	    
            NomGaitSpeed = 85


def GetCurrentTime():
	lCurrentTime = datetime.datetime.now()
	return lCurrentTime


def ReadServoOffsets():     #read finetuneoffset from file ... maby later
    for Index in range(0,32):
        aServoOffsets.append(0)



#--------------------------------------------------------------------
#[FREE SERVOS] Frees all the servos
def FreeServos():
	hservo([cRFCoxaPin,-30000], 
			[cRFFemurPin,-30000], 
			[cRFTibiaPin,-30000], 
			[cRMCoxaPin,-30000], 
			[cRMFemurPin,-30000], 
			[cRMTibiaPin,-30000], 
	 		[cRRCoxaPin,-30000], 
			[cRRFemurPin,-30000], 
	  		[cRRTibiaPin,-30000], 
			[cLFCoxaPin,-30000], 
	  		[cLFFemurPin,-30000], 
			[cLFTibiaPin,-30000], 
	  		[cLMCoxaPin,-30000], 
			[cLMFemurPin,-30000], 
	  		[cLMTibiaPin,-30000], 
			[cLRCoxaPin,-30000], 
	 		[cLRFemurPin,-30000], 
			[cLRTibiaPin,-30000])
	fFirstMove = 1		# May need to tell servo system again to do it without time values...


	
#==============================================================================
def GetAD(cannel):
    if debug: print(y)
    for x in range(0,samples):
      List1 = []
      OneTime = adc.readADCSingleEnded(cannel, gain, 128)
      List1.append(OneTime)
    #Samples ready, go for the Mid
    total=0
    for z in range(0,len(List1)):
        total += List1[z]
    if debug: print(total)
    #MidListInt.append(total/len(List1))
    return int(round(total/len(List1)))


def GetLiPoVoltage() :
    global samples
    global LiPoLowVoltage
    global LiPoReading
    samples = 1
    reading = GetAD(0)
    LiPoReading = int(round(reading,0))*2
    if debug : print("LiPoReading: ",LiPoReading)
    if ((LiPoReading < LiPoShutOffVoltage) and (hexon == 1)) :
        LiPoLowVoltage = 1
        buzz([[800,200],[700,200],[600,200],[500,200],[400,200],[300,200],[200,2000]]) # Major Warning beeper!
        parklegs()
        HexOn = 0
    if LiPoLowVoltage == 1 :
        buzz([[300,100],[200,100]])  # pitch and duration
        print("Low LiPoReading!! : ",LiPoReading)  
        HexOn = 0
    return


def ParkLegs():
    global SSCTime
    global FemurAngle1
    global TibiaAngle1
    global CoxaAngle1
    print("parking legs")
    for LegIndex in range(0,6):
        FemurAngle1[LegIndex] = -1800
        TibiaAngle1[LegIndex] = -1800
        CoxaAngle1[LegIndex] = 0
    if debug : 
        print(FemurAngle1)
        print(TibiaAngle1)
        print(CoxaAngle1)
    SSCTime = 600                   # Nice and slow
    CheckAngles()
    ServoDriverStart()
    ServoDriverCommit()
    hservowait([cRFCoxaPin,	cRFFemurPin, cRFTibiaPin,
        cRMCoxaPin, cRMFemurPin, cRMTibiaPin,
        cRRCoxaPin, cRRFemurPin, cRRTibiaPin,
        cLFCoxaPin, cLFFemurPin, cLFTibiaPin,
        cLMCoxaPin, cLMFemurPin, cLMTibiaPin,
        cLRCoxaPin, cLRFemurPin, cLRTibiaPin])


def servoWrite(channel , pulsein):
    #servosaver !!!
    if debug : print("channel: %d pulsin %d" % (channel, pulsein))
    snr,fss,minpuls,maxpuls,speed,endpos = serviDynMa[channel]
    if ((pulsein < minpuls) or (pulsein > maxpuls)) : 
        print("ERROR channel: %d pulsin %d" % (channel, pulsein))
        raise SystemError
    else:
        if channel in range(0,15):
            #pwm.setPWM(channel,0, int(pulsein/pulseConstant))   
            time.sleep(0.001)            
        else:
            #pwm2.setPWM(channel-16,0, int(pulsein/pulseConstant)) 
            time.sleep(0.001)                           


def hservosync(PinPosSpeedTulip,delay): #([[1,10500],[2,-3000]],150)    #delay in ms
    ##-12000 to 12000 expected input 
    global servoDynTh
    global serviDynMa
    global BlockServoUpdates
    #timetulip  = []
    #speedtulip = []
    traveltime = 0
    traveltimemax = 0
    for PinPosSpeed in PinPosSpeedTulip:  #first round, to get max traveltime
        #unpack
        snrin,endposin = PinPosSpeed
        #if debug: print("snrin: ",snrin)
        snr,fss,minpuls,maxpuls,speed,endpos = serviDynMa[snrin]
        snr,posnow = servoDynTh[snrin]
        if minpuls == 0 : raise SystemError         #error in the configuration
        if debug: 
            print("snr: ",snr)
            print("endposin1: ",endposin)
        if endposin > 12000 : endposin = 12000
        if endposin < -12000 : endposin = -12000
        endposin = int(round(interp(endposin,[-12000,12000],[minpuls,maxpuls])))
        if debug: print("endposin2: ",endposin)
        if posnow != endposin: 
            if debug: print("posnow: ",posnow)
            if debug: print("endposin: ",endposin)
            traveltime = (fss*1.0)/(maxpuls-minpuls)*abs(posnow - endposin)
            if debug: print("traveltime: ",traveltime)
            if traveltime > traveltimemax : traveltimemax = traveltime
        if debug: print("PinPosSpeed1: ",PinPosSpeed)
        PinPosSpeed = snrin,endposin    #pack pulse so we dont need to do the calc again       
        if debug: print("PinPosSpeed2: ",PinPosSpeed)

    if traveltimemax > 0:       # at least one of the servoes has a distance to travel, go for second round
        if debug: 
            print("traveltimemax: ",traveltimemax)
            print("delay: ",delay)
        if delay in range(0,2000): traveltimemax += (delay/1000.0)
        if debug: print("traveltimemax (delay included): ",traveltimemax)
        BlockServoUpdates = 1                   # set flag for exclusive servo update
        while ServosBeingUpdated == 1:          # wait for the returnflag to be set
            if debug: print("waiting for ServosBeingUpdated == 1") 
            time.sleep(0.0001)      
        
        for PinPosSpeed in PinPosSpeedTulip:    #second round, to use traveltimemax to set speed
            #unpack
            snrin,endposin = PinPosSpeed

            snr,fss,minpuls,maxpuls,speed,endpos = serviDynMa[snrin]
            snr,posnow = servoDynTh[snrin]

            if endposin > 12000 : endposin = 12000
            if endposin < -12000 : endposin = -12000
            endposin = int(round(interp(endposin,[-12000,12000],[minpuls,maxpuls])))
            if debug: print("endposin2: ",endposin)

            speedin = int(round((abs(endposin-posnow))/(traveltimemax*driverHz)))     # Integer please
            #pack
            
            serviDynMa[snr] = snr,fss,minpuls,maxpuls,speedin,endposin
            
            if debug: print(serviDynMa[snr])
        BlockServoUpdates = 0                   # unset flag for exclusive servo update
    elif debug: print("traveltimeMax = 0")




def hservo(PinPosSpeedTulip): #([[1,-10000,10],[2,12000,20]]) 
    ##-12000 to 12000 expected input => ((input/12)+1500)
    ## if input = -30000 then input = 0 (free servo)
    global servoDynTh
    global serviDynMa
    for PinPosSpeed in PinPosSpeedTulip:
        if len(PinPosSpeed)==3 :
            snrin,endposin,speedin = PinPosSpeed
        else :
            snrin,endposin = PinPosSpeed
            speedin = 9999                  # if only two parameters => full speed ahead !
        if speedin == 0 : speedin = 1       # dont accept zero speed, it will cause a deathloop
        snr,fss,minpuls,maxpuls,speed,endpos = serviDynMa[snrin]
        if minpuls == 0 : raise SystemError         #error in the configuration
        snr,posnow = servoDynTh[snrin]
        #if debug: print(speedin)
        if speedin > ((maxpuls-minpuls)/(fss*driverHz)) : 
            speedin = ((maxpuls-minpuls)/(fss*driverHz))
        speedin = int(round(speedin))
        if debug: print(speedin)
        if endposin == -30000:
            endposin = 0
        else:
            if endposin > 12000 : endposin = 12000
            if endposin < -12000 : endposin = -12000
            if debug: print(endposin)
            endposin = int(round(interp(endposin,[-12000,12000],[minpuls,maxpuls])))
            if debug: print(endposin)
        if debug: print(endposin)
        serviDynMa[snrin] = snr,fss,minpuls,maxpuls,speedin,endposin
        #print(serviDynMa[snrin])

def hservowait(PinList):    #([1,5,6]) The HSERVOWAIT command is used to delay program execution until HSERVO has finished updating the servos specified
    global servoDynTh
    global serviDynMa
    if debug : print("waiting for ",PinList)
    for Pin in PinList:
        snr,fss,minpuls,maxpuls,speed,endpos = serviDynMa[Pin]
        snr,posnow = servoDynTh[Pin]
        while posnow != endpos : 
            if debug : print("Now waiting for snr ", snr, str(abs(posnow - endpos)))
            snr,fss,minpuls,maxpuls,speed,endpos = serviDynMa[Pin]
            snr,posnow = servoDynTh[Pin]
            time.sleep(1.0/driverHz)
    if debug : print("waiting done")

def hservopos(pin): #(1) It will return the position last given to a specified servo
    ## OBS +-12000
    global serviDynMa
    snr,fss,minpuls,maxpuls,speed,endpos = serviDynMa[pin]
    return (endpos-1500)*12

def hservoposNow(pin): #(1) It will return the calculated position of a specified servo
    ## OBS +-12000
    snr,posnow = servoDynTh[pin]
    return (posnow-1500)*12

def hservoidle(pin): #(1) return 1 if idle, else 0
    global servoDynTh
    global serviDynMa
    snr,fss,minpuls,maxpuls,speed,endpos = serviDynMa[pin]
    snr,posnow = servoDynTh[pin]
    returnint = 1
    if posnow != endpos : 
        returnint = 0
    return returnint


def buzz(pitchandduration):
        buzzer = BuzzDriver(pitchandduration)
        buzzer.start()

class BuzzDriver(threading.Thread):
    def __init__(self, pitchanddurationTulip):
        threading.Thread.__init__(self)
        self.pitchanddurationTulip = pitchanddurationTulip   # Pairs of Pitch in Hz and durition in ms
        #self.duration = duration    # in ms
        self.running = False
    
    def run(self):
        self.running = True
        for Pair in self.pitchanddurationTulip:
            pitch,duration = Pair
            period = 1.0 / pitch
            delay = period / 2.0
            cycles = int((duration/1000.0) * pitch)
            for i in range(cycles):
                if self.running == False : break
                GPIO.output(buzzer_pin, True)
                time.sleep(delay)
                GPIO.output(buzzer_pin, False)
                time.sleep(delay)

    def stop(self):
        self.running = False



class servoDriver(threading.Thread):
    def __init__(self, cps):
        threading.Thread.__init__(self)
        self.cps = cps
        self.running = False

    def run(self):
        self.running = True
        global servoDynTh
        global serviDynMa
        global ServosBeingUpdated
        while (self.running):
            while BlockServoUpdates == 1:   #do NOT update servos during hservosync
                if debug : print("waiting for BlockServoUpdates") 
                time.sleep(0.0001)          
            ServosBeingUpdated = 1          #set flag "now am i updating servos"
            a = datetime.datetime.now()
            #Dostuff
            for x in range (0,noServo):
                if self.running == False :break
                snr,posnow = servoDynTh[x]
                #if debug : print("servodriver snr %d" % snr)
                snr,fss,minpuls,maxpuls,speed,endpos = serviDynMa[x]
                #if debug : 
                #    if (posnow == endpos):
                #        print("servodriver snr %d posnow == endpos %d" % (snr,posnow))
                if (posnow < endpos):   #going bigger
                    if ((posnow + speed) > endpos):
                        posnow = endpos
                    else: posnow += speed
                    if debug: print("servodriver (bigger) posnow: ",posnow)
                    servoWrite(x,posnow)        #tar ca 0,003s
                elif (posnow > endpos): #going smaller
                    if ((posnow - speed) < endpos):
                        posnow = endpos
                    else: posnow -= speed
                    if debug: print("servodriver (smaller) posnow: ",posnow)
                    servoWrite(x,posnow)        #tar ca 0,003s
                servoDynTh[x] = snr,posnow
            #Done
            ServosBeingUpdated = 0
            b = datetime.datetime.now()
            delta = b - a
            deltatime = delta.total_seconds()+0.000156
            #print("deltatime=",deltatime," and cps=",self.cps)
            if deltatime > self.cps: print("Overtime!! deltatime=",deltatime," and cps=",self.cps)
            else : time.sleep(self.cps-deltatime)
        self.running = False

    def stop(self):
        self.running = False

def InitController():
    global LastListInByte
    #ListInByteOK = []
    LastListInByte = [0,0,0,0,0,0,0]


class ReadController(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.running = False

    def run(self):
        self.running = True
        global ListInByteOK
        while (self.running):
            a = datetime.datetime.now()
            #Dostuff
            #if debug : print("waiting for radiotrafic")
            rcv = port.read(1)
            if rcv == chr(170):       # strip of first checkbyte
                rcv = port.read(1)
                if rcv == chr(170):     # strip of second checkbyte
                    ListInByte = []
                    for x in range(0,7):    
                        ListInByte.append(ord(port.read(1)))
                    check = int(ListInByte[6]) ^ int(ListInByte[5])  #check the checksum
                    for x in range(4,-1,-1): 
                        check = int(ListInByte[x]) ^ (check) 
                    #if debug: print(check) 
                    if check == 170:
                        #if debug: print("good checksum")        # don't do ANYTHING without a good checksum !!!
                        # Do the stuff
                        ListInByteOK = ListInByte #transfer local good data to globalbuffer
            #Done
            b = datetime.datetime.now()
            delta = b - a
            deltatime = delta.total_seconds()
            #if debug: print("ReadController cycletime", deltatime)
        self.running = False

    def stop(self):
        self.running = False


class DisplayDriver(threading.Thread):
    def __init__(self, queue):
        threading.Thread.__init__(self)
        self.queue = Queue()
        self.running = False 
    
    def run(self):   
        self.running = True
        if debug : print("init display")
        # Load default font.
        font = ImageFont.load_default()
        font2 = ImageFont.truetype("ARIALN.TTF", 20)
        
        # choose active display
        device = ssd1306(port=1, address=0x3C)			
        #device = sh1106(port=1, address=0x3C)

        # Create image buffer.
        width = device.width
        height = device.height
 
        #----------------------------------------------------------------------------------------------------------
        rowhight=12
        while (self.running):
            a = datetime.datetime.now()
            if debug: print("Que start")
            GetLiPoVoltage()
            with canvas(device) as draw:
                if self.running == False : break
                a = LiPoReading/1000.0
                draw.text((70,40), ("%.3f V" % a), font=font2, fill=255)
            time.sleep(1)
        self.running = False


    def stop(self):
        self.running = False



#    B0:    JL_updn     Dualshock(6)
#    B1:    JL_ltrt     Dualshock(5)
#    B2:    JR_updn     Dualshock(4)
#    B3:    JR_ltrt     Dualshock(3)
#    B4:    button=0-7
#       j_LefL3     = 4,1       0
#       j_RigR3     = 4,2       1
#       b_R1        = 4,4       2
#       b_R2        = 4,8       3
#       b_L2        = 4,16      4
#       b_L1        = 4,32      5
#       b_Select    = 4,64      6
#       b_Start     = 4,128     7
#    B5:    button=8-15
#       p_Tri       = 5,1       0
#       p_Cir       = 5,2       1
#       p_X         = 5,4       2
#       p_Squ       = 5,8       3
#       p_Up        = 5,16      4
#       p_Rig       = 5,32      5
#       D_Dow       = 5,64      6
#       p_Lef       = 5,128     7

#       endpos = (int(ListOKByte[0]) * ((maxpuls-minpuls)/256)) + minpuls
#       speed = ((maxpuls-minpuls)/(fss*driverHz))/255 * int(ListOKByte[3])

if __name__ == "__main__": 
    print("Starting servodriver")
    servoD = servoDriver(1.0/driverHz)
    print("Starting reciever")
    xbeeIn = ReadController()
    try:
        print("Starting threads")
        servoD.start()
        xbeeIn.start()
        
        #time.sleep(0.5) #to start up xbee
        # Get the servo offsets
        ReadServoOffsets() #read finetuneoffset from file
        GaitSelect() # choose a gate ##
        if debug: print("Setting legs in startpostion")
        ParkLegs() # including hservowait
        #time.sleep(100000)
        while True: #main loop
            #Start time
            lTimerStart = datetime.datetime.now() 
            if debug: print("Starting mainloop")
            while ListInByteOK == []:    
                print("Controller is not responding")
                time.sleep(1)
            #ListInByteOld = ListInByte  #save old value
            ListInByte = ListInByteOK   #new data is in ListInByteOK
            
            if ((ListInByte[4] & (1 << 7)) > 0) and ((LastListInByte[4] & (1 << 7)) == 0)  : # button start is pressed
                if debug: print("Startbutton pressed")
                if HexOn==1: 
                    if debug: print("Bot is on, therefor going off")
                    BodyPosX = 0
                    BodyPosY = 0
                    BodyPosZ = 0
                    BodyRotX1 = 0
                    BodyRotY1 = 0
                    BodyRotZ1 = 0
                    TravelLengthX = 0
                    TravelLengthZ = 0
                    TravelRotationY = 0
                    BodyYOffset = 0
                    BodyYShift = 0
                    SelectedLeg = 255
                    HexOn = 0
                else:
                    if debug: print("Bot is off, therefor going on")
                    HexOn = 1   #It's off. turn it on	

            if HexOn == 1: #the rest is ONLY done if bot is on !
                #Translate mode
                if ((ListInByte[4] & (1 << 4)) > 0) and ((LastListInByte[4] & (1 << 4)) == 0)    : # button L1 is pressed
                    if debug: print("Button L1 is presed: Translatemode")
                    buzz([[500,300],[700,100]])  # pitch and duration
                    if ControlMode != TranslateMode:
                        ControlMode = TranslateMode
                    else:
                        if (SelectedLeg==255):
                            ControlMode = WalkMode
                        else:
                            ControlMode = SingleLegMode
                #Rotate mode
                if ((ListInByte[4] & (1 << 5)) > 0) and ((LastListInByte[4] & (1 << 5)) == 0) : # button L2 is pressed
                    if debug: print("Button L2 is presed: Rotate mode")
                    buzz([[500,300],[700,100]])  # pitch and duration
                    if ControlMode != RotateMode:
                        ControlMode = RotateMode
                    else:
                        if (SelectedLeg==255):
                            ControlMode = WalkMode
                        else:
                            ControlMode = SingleLegMode
                #Single leg mode
                if ((ListInByte[5] & (1 << 1)) > 0) and ((LastListInByte[5] & (1 << 1)) == 0) : # button Circle is pressed
                    if debug: print("Button Circle is pressed: Single leg mode")
                    if abs(TravelLengthX)<cTravelDeadZone and abs(TravelLengthZ)<cTravelDeadZone and abs(TravelRotationY*2)<cTravelDeadZone:
                        buzz([[500,300],[700,100]])  # pitch and duration
                        if (ControlMode != SingleLegMode):
                            ControlMode = SingleLegMode
                            if (SelectedLeg == 255): #Select leg if none is selected
                                SelectedLeg=cRF #Startleg
                        else:
                            ControlMode = WalkMode
                            SelectedLeg=255	      
                #Not allocates button
                if ((ListInByte[5] & (1 << 2)) > 0) and ((LastListInByte[5] & (1 << 2)) == 0) : # button X is pressed
                    if debug: print("Button X is presed: Not Allocated :)")
                    time.sleep(0)
                #[Common functions]
                #Switch Balance mode on/off
                if ((ListInByte[5] & (1 << 3)) > 0) and ((LastListInByte[5] & (1 << 3)) == 0) : # button Square is pressed
                    if debug: print("Button Square is presed: Switch Balance mode on/off")
                    if BalanceMode == 1:
                        BalanceMode = 0
                    else: BalanceMode = 1
                    if BalanceMode :
                        buzz([[500,300],[700,100]])  # pitch and duration	  
                    else:	  
                        buzz([[700,300],[300,100]])  # pitch and duration
                #Stand up, sit down
                if ((ListInByte[5] & (1 << 0)) > 0) and ((LastListInByte[5] & (1 << 0)) == 0) : # button Triangle is pressed
                    if (BodyYOffset>0):
                        BodyYOffset = 0
                    else:
                        BodyYOffset = 50
                #D-Up Button test
                if ((ListInByte[5] & (1 << 4)) > 0) and ((LastListInByte[5] & (1 << 4)) == 0): # D-up up is pressed
                    if debug: print("Button D-pad up: Raise body")
                    BodyYOffset = BodyYOffset+10
                if ((ListInByte[5] & (1 << 6)) > 0) and ((LastListInByte[5] & (1 << 6)) == 0) : # D-Dow up is pressed
                    if debug: print("Button D-pad down: lower body")
                    BodyYOffset = BodyYOffset-10	
                if ((ListInByte[5] & (1 << 5)) > 0) and ((LastListInByte[5] & (1 << 5)) == 0) : # D-Rig is pressed
                    if debug: print("Button D-pad right:")
                    if SpeedControl > 0:                                # lower value for SpeedControl  => faster
                        if debug: print("Going faster")
                        SpeedControl = SpeedControl - 50
                        if SpeedControl < 0 : SpeedControl = 0
                    buzz([[500,300],[700,100]])  # pitch and duration
                if ((ListInByte[5] & (1 << 7)) > 0) and ((LastListInByte[5] & (1 << 7)) == 0) : # D-Lef is pressed
                    if debug: print("Button D-pad left:")
                    if SpeedControl<2000 :
                        if debug: print("Going slower")
                        SpeedControl = SpeedControl + 50
                        if SpeedControl > 2000 : SpeedControl = 2000
                    buzz([[500,300],[700,100]])  # pitch and duration	  

                #[Walk functions]
                if (ControlMode==WalkMode):
                    #Switch gates
                    if ((ListInByte[4] & (1 << 6)) > 0) and ((LastListInByte[4] & (1 << 6)) == 0) and  abs(TravelLengthX)<cTravelDeadZone and abs(TravelLengthZ)<cTravelDeadZone and abs(TravelRotationY*2)<cTravelDeadZone:
                        # Select Button is pressed and no movemwnt
                        if debug: print("Select Button is pressed and no movemwnt")
                        if GaitType<7:
                            buzz([[500,300],[700,100]])  # pitch and duration
                            GaitType = GaitType+1
                        else:
                            buzz([[700,300],[500,100]])  # pitch and duration
                            GaitType = 0
                        if debug: print("Gait selected: ",GaitType)
                        GaitSelect()					
                    #Double leg lift height		
                    if ((ListInByte[4] & (1 << 2)) > 0) and ((LastListInByte[4] & (1 << 2)) == 0):    # R1 is pressed
                        buzz([[500,300],[700,100]])  # pitch and duration
                        if DoubleHeightOn == 1 :
                            DoubleHeightOn = 0
                        else:
                            DoubleHeightOn = 1
                        if DoubleHeightOn == 1:
                            LegLiftHeight = 80
                        else:
                            LegLiftHeight = 50
                    #Double Travel Length
                    if ((ListInByte[4] & (1 << 3)) > 0) and ((LastListInByte[4] & (1 << 3)) == 0):    # R2 is pressed
                        buzz([[500,300],[700,100]])  # pitch and duration
                        if DoubleTravelOn == 1:
                            DoubleTravelOn = 0
                        else:
                            DoubleTravelOn = 1
                    # Switch between Walk method 1 and Walk method 2
                    if ((ListInByte[4] & (1 << 1)) > 0) and ((LastListInByte[4] & (1 << 1)) == 0):    # R3 is pressed
                        buzz([[500,300],[700,100]])  # pitch and duration
                        if WalkMethod == 1:
                            WalkMethod = 0
                        else:
                            WalkMethod = 1
                    #Walking
                    if WalkMethod == 1 : #(Walk Method)
                        TravelLengthZ = (ListInByte[2] - 128) #Right Stick Up/Down
                    else:
                        TravelLengthX = -(ListInByte[1] - 128)        
                        TravelLengthZ = (ListInByte[1] - 128)    
                    if DoubleTravelOn == 0:         
                        TravelLengthX = TravelLengthX/2
                        TravelLengthZ = TravelLengthZ/2
                    TravelRotationY = -(ListInByte[3] - 128)/4 #Right Stick Left/Right       	  
                #[Translate functions]
                #BodyYShift = 0
                if (ControlMode==TRANSLATEMODE) :
                    BodyPosX = (ListInByte[1] - 128)/2
                    BodyPosZ = -(ListInByte[0] - 128)/3
                    BodyRotY1 = (ListInByte[3] - 128)/6
                    BodyYShift = (-(ListInByte[2] - 128)/2)
                #[Rotate functions]
                if (ControlMode==ROTATEMODE):
                    BodyRotX1 = (ListInByte[0] - 128)/8
                    BodyRotY1 = (ListInByte[3] - 128)/6
                    BodyRotZ1 = (ListInByte[1] - 128)/8
                    BodyYShift = (-(ListInByte[2] - 128)/2)
                #Single Leg Mode
                if (ControlMode==SINGLELEGMODE):
                    #Switch leg for single leg control
                    if ((ListInByte[4] | (1 << 6)) > 0):	#Select Button test 
                        buzz([[500,300],[700,100]])  # pitch and duration
                        if SelectedLeg < 5:
                            SelectedLeg = SelectedLeg+1
                        else:
                            SelectedLeg = 0
                    SLLegX	= (ListInByte[1] - 128)/2 #Left Stick Right/Left
                    SLLegY	= (ListInByte[2] - 128)/10 #Right Stick Up/Down
                    SLLegZ 	= (ListInByte[0] - 128)/2 #Left Stick Up/Down
                    # Hold single leg in place
                    if ((ListInByte[4] | (1 << 3)) > 0):    # R2 is pressed
                        buzz([[500,300],[700,100]])  # pitch and duration
                        if SLHold == 1:
                            SLHold = 0
                        else:
                            SLHold = 1
                #Calculate walking time delay
                InputTimeDelay = 128 - (abs(ListInByte[1] - 128)) 
                if InputTimeDelay < abs((ListInByte[0] - 128)) : InputTimeDelay = abs((ListInByte[0] - 128))
                if InputTimeDelay < abs((ListInByte[3] - 128)) : InputTimeDelay = abs((ListInByte[3] - 128))
            #Calculate BodyPosY
            BodyPosY = (BodyYOffset + BodyYShift) 
            if BodyPosY < 0 : BodyPosY = 0   
            #Store previous state
            LastListInByte = ListInByte
            #print(LegPosY[cRF])
            #print(cInitPosY[cRF])
            #print(LegPosY[cRM])
            #print(cInitPosY[cRM])
            #print(LegPosY[cRR])
            #print(cInitPosY[cRR])
            #print(LegPosY[cLR])
            #print(cInitPosY[cLR])
            #print(LegPosY[cLM])
            #print(cInitPosY[cLM])
            #print(LegPosY[cLF])
            #print(cInitPosY[cLF])
            #SingleLegControl:
            AllDown = LegPosY[cRF]==cInitPosY[cRF] and LegPosY[cRM]==cInitPosY[cRM] and LegPosY[cRR]==cInitPosY[cRR] and LegPosY[cLR]==cInitPosY[cLR] and LegPosY[cLM]==cInitPosY[cLM] and LegPosY[cLF]==cInitPosY[cLF]
            if (SelectedLeg>=0 and SelectedLeg<=5):
                if (SelectedLeg!=Prev_SelectedLeg):
                    if (AllDown) :  #Lift leg a bit when it got selected
                        LegPosY[SelectedLeg] = cInitPosY[SelectedLeg]-20  
                        #Store current status
                        Prev_SelectedLeg = SelectedLeg	         
                    else:
                        #Return prev leg back to the init position
                        LegPosX[Prev_SelectedLeg] = cInitPosX[Prev_SelectedLeg]
                        LegPosY[Prev_SelectedLeg] = cInitPosY[Prev_SelectedLeg]
                        LegPosZ[Prev_SelectedLeg] = cInitPosZ[Prev_SelectedLeg]
                elif (not SLHold):
                    LegPosY[SelectedLeg] = LegPosY[SelectedLeg]+SLLegY
                    LegPosX[SelectedLeg] = cInitPosX[SelectedLeg]+SLLegX
                    LegPosZ[SelectedLeg] = cInitPosZ[SelectedLeg]+SLLegZ      
            else: #All legs to init position
                if (not AllDown):
                    for LegIndex in range(0,6) :
                        LegPosX[LegIndex] = cInitPosX[LegIndex]
                        LegPosY[LegIndex] = cInitPosY[LegIndex]
                        LegPosZ[LegIndex] = cInitPosZ[LegIndex]
                if Prev_SelectedLeg!=255:
                    Prev_SelectedLeg = 255






            #[GAIT Sequence] Calculate Gait sequence
            LastLeg = 0
            for LegIndex in range(0,6) : # for all legs
                if LegIndex == 5 : LastLeg = 1 #last leg
                Gait(LegIndex) 
            #Balance calculations
            TotalTransX = 0 #reset values used for calculation of balance
            TotalTransZ = 0
            TotalTransY = 0
            TotalXBal1 = 0
            TotalYBal1 = 0
            TotalZBal1 = 0
            if (BalanceMode>0):
                for LegIndex in range(0,3):	# balance calculations for all Right legs
                    BalCalcOneLeg([-LegPosX(LegIndex)+GaitPosX(LegIndex), 
					    LegPosZ(LegIndex)+GaitPosZ(LegIndex), 
						LegPosY(LegIndex)-cInitPosY(LegIndex)+GaitPosY(LegIndex), 
						LegIndex])
                for LegIndex in range(3,6):	# balance calculations for all Left legs
                    BalCalcOneLeg( [LegPosX(LegIndex)+GaitPosX(LegIndex), 
					    LegPosZ(LegIndex)+GaitPosZ(LegIndex), 
						LegPosY(LegIndex)-cInitPosY(LegIndex)+GaitPosY(LegIndex), 
						LegIndex])
            BalanceBody()
            #Reset IKsolution indicators 
            IKSolution = 0 
            IKSolutionWarning = 0 
            IKSolutionError = 0 
            fDebugRotDisp = 0
            #if (wDebugLevel and DBG_LVL_NORMAL):
            #    if BodyRotZ1 and (BodyRotZ1 != xxxLastBodyRotZ1):
            #        fDebugRotDisp = 1
            xxxLastBodyRotZ1 = BodyRotZ1
            #Do IK for all Right legs
            for LegIndex in range(0,3):	
                BodyIK(-LegPosX[LegIndex]+BodyPosX+GaitPosX[LegIndex] - TotalTransX, 
	  				LegPosZ[LegIndex]+BodyPosZ+GaitPosZ[LegIndex] - TotalTransZ, 
	  				LegPosY[LegIndex]+BodyPosY+GaitPosY[LegIndex] - TotalTransY, 
	  				GaitRotY[LegIndex], LegIndex)
                LegIK(LegPosX[LegIndex]-BodyPosX+BodyIKPosX-(GaitPosX[LegIndex] - TotalTransX), 
	  				LegPosY[LegIndex]+BodyPosY-BodyIKPosY+GaitPosY[LegIndex] - TotalTransY, 
	  				LegPosZ[LegIndex]+BodyPosZ-BodyIKPosZ+GaitPosZ[LegIndex] - TotalTransZ, LegIndex)   
            #Do IK for all Left legs  
            for LegIndex in range(3,6):	
                BodyIK(LegPosX[LegIndex]-BodyPosX+GaitPosX[LegIndex] - TotalTransX, 
	  				LegPosZ[LegIndex]+BodyPosZ+GaitPosZ[LegIndex] - TotalTransZ, 
	  				LegPosY[LegIndex]+BodyPosY+GaitPosY[LegIndex] - TotalTransY, 
	  				GaitRotY[LegIndex], LegIndex)
                LegIK(LegPosX[LegIndex]+BodyPosX-BodyIKPosX+GaitPosX[LegIndex] - TotalTransX, 
	  				LegPosY[LegIndex]+BodyPosY-BodyIKPosY+GaitPosY[LegIndex] - TotalTransY, 
	  				LegPosZ[LegIndex]+BodyPosZ-BodyIKPosZ+GaitPosZ[LegIndex] - TotalTransZ, LegIndex)
            #Return to the middle position 
            BodyAngle=0
            #IF ((abs(TravelLengthX)>TravelDeadZone | abs(TravelLengthZ)>TravelDeadZone)  & TravelLengthZ<TravelDeadZone) | abs(TravelRotationY*10)>TravelDeadZone THEN 
            #IF 1 then #((abs(TravelLengthX) > cTravelDeadZone) OR (abs(TravelLengthZ)>cTravelDeadZone)) THEN 
            #Calculate walking direction X and Z 
            TravelLengthXZ = SQR((TravelLengthX * TravelLengthX) + TravelLengthZ * TravelLengthZ)
            BodyAngle = TOINT(FACOS(TOFLOAT(TravelLengthZ) / TOFLOAT(TravelLengthXZ)) * 180.0 / 3.141592)-180
            #Add sign depending on the direction of X 
            BodyAngle = BodyAngle * (TravelLengthX/abs(TravelLengthX)) 
            #ENDIF
            #Calculate body angle depending on rotation
            #IF 1 then #((abs(TravelRotationY*2)>cTravelDeadZone) AND (abs(TravelRotationY*3) > abs(BodyAngle))) THEN
            BodyAngle = -TravelRotationY*2 # Rotation max = 16*6 to get max range of 90 deg.   
            #ENDIF 
            #hserout ["BodyAngle: ", sdec BodyAngle]
            #hserout [" TravelLengthXZ: ", sdec TravelLengthXZ, 13]
            
            #if TravelLengthXZ == 0:
            #    Headtilt = cHeadTiltFast1
            #else:
            #    Headtilt = cHeadTiltSlow1 - (3 * TravelLengthXZ) 

            if fDebugRotDisp: 
                print("IKS W/E", hex(IKSolutionWarning), " ", hex(IKSolutionError), 13)

            #Write IK errors to leds
            #LedC = IKSolutionWarning
            #LedA = IKSolutionError

            #Drive Servos
            if HexOn :  
                if HexOn and Prev_HexOn==0:
                    print("HexOn AND Prev_HexOn=0")
                    #Sound cSound,[60\4000,80\4500,100\5000]
                    #Eyes = 1
                    #Set SSC time
                    if (abs(TravelLengthX)>cTravelDeadZone or abs(TravelLengthZ)>cTravelDeadZone or abs(TravelRotationY*2)>cTravelDeadZone):
                        SSCTime = NomGaitSpeed + (InputTimeDelay*2) + SpeedControl
                    #Add aditional delay when Balance mode is on
                    if BalanceMode :
                        SSCTime = SSCTime + 100                     # lower value for SpeedControl  => faster
                    else: #Movement speed excl. Walking
                        SSCTime = 200 + SpeedControl
                    # note we broke up the servo driver into start/commit that way we can output all of the servo information
                    # before we wait and only have the termination information to output after the wait.  That way we hopefully
                    # be more accurate with our timings...
                    ServoDriverStart()

                    #Sync BAP with SSC while walking to ensure the prev is completed before sending the next one
                    GaitPeak = 0 #Reset
                    # Finding any incident of GaitPos/Rot != 0:
                    for LegIndex in range(0, 6):
                        if GaitPeak < abs(GaitPosX(LegIndex)):
                            GaitPeak = abs(GaitPosX(LegIndex))
                        elif GaitPeak < abs(GaitPosY(LegIndex)):
                            GaitPeak = abs(GaitPosY(LegIndex))
                        elif GaitPeak < abs(GaitPosZ(LegIndex)):
                            GaitPeak = abs(GaitPosZ(LegIndex))
                        elif GaitPeak < abs(GaitRotY(LegIndex)):
                            GaitPeak = abs(GaitRotY(LegIndex))
                    if (GaitPeak > 2)  or Walking:  #if GaitPeak is higher than 2 the robot are still walking
                        Walking = (GaitPeak > 2)		# remember why we came in here
                    #Get endtime and calculate wait time
                    lTimerEnd = GetCurrentTime()
                    CycleTime = (lTimerEnd-lTimerStart)
                    #Wait for previous commands to be completed while walking
                    #if (PrevSSCTime > CycleTime) time.sleep(PrevSSCTime - CycleTime) #   Min 1 ensures that there alway is a value in the pause command  
                    HSERVOWAIT([cRFCoxaPin,	cRFFemurPin, cRFTibiaPin,
						cRMCoxaPin, cRMFemurPin, cRMTibiaPin,
						cRRCoxaPin, cRRFemurPin, cRRTibiaPin,
						cLFCoxaPin, cLFFemurPin, cLFTibiaPin,
						cLMCoxaPin, cLMFemurPin, cLMTibiaPin,
						cLRCoxaPin, cLRFemurPin, cLRTibiaPin])
                    ServoDriverCommit()
                else: 
                    #Turn the bot off
                    if (Prev_HexOn or not AllDown):
                        SSCTime = 600
                        ServoDriverStart()
                        ServoDriverCommit()  		
                        #Sound cSound,[100\5000,80\4500,60\4000]      
                        time.sleep(0.6)
                    else:   
                        print("Going to Park Legs")
                        ParkLegs()	#*nc
                        #GOSUB FreeServos
                        #Eyes = 0
                        #Only when the robot is not active do I do the Terminal monitor...
                        #gosub TerminalMonitor[0]
                        #ENDIF	
  
                #Store previous HexOn State
                if HexOn:
                    Prev_HexOn = 1
                else:
                    Prev_HexOn = 0
  







    


    # errorhandeling:
    except KeyboardInterrupt:
        print("Cancelled")
    #exceptions.TypeError
    except:
        print("Unexpected error:", sys.exc_info()[0])
        raise
    finally:
        print("Stopping threads ...")
        servoD.stop()
        xbeeIn.stop()
        #buzzer.stop()
        print("waiting for the threads to finish if they hasn't already")
        servoD.join()
        xbeeIn.join()
        #buzzer.join()
        print("cleaning up GPIO")
        GPIO.cleanup()
    print("Done")
    # END def main():

#if __name__ == "__main__": 
#    main()
