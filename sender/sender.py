#!/usr/bin/python

#print("Loading ptvsd")
#import ptvsd
#ptvsd.enable_attach(secret = "pi")
#a = raw_input("Connect debug pi@192.168.1.245 and press to start")

print("Loading serial, time, sys, threading, datetime, signal, os.path")
import serial, time, sys, threading, datetime, signal, os.path
print("Loading ADS")
from Adafruit_ADS1x15 import ADS1x15
print("Loading Numpy")
from numpy import interp
print("Loading wiringpi2")
import wiringpi2 as wiringpi 
print("ConfigParser")
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

#init
debug = False
queue = Queue()

#Function declaration:

def scanButtons():
  button = 0x0
  for x in range(0,8):
    if not wiringpi.digitalRead(pin_base+x):
      button = (button | (1 << x))
  LSB = button
  button = 0x0
  for x in range(8,16):
    if not wiringpi.digitalRead(pin_base+x):
      button = (button | (1 << (x-8)))
  MSB = button
  return LSB,MSB


def GetAD(cannel):
    if debug: print(y)
    for x in range(0,samples):
      List1 = []
      OneTime = adc.readADCSingleEnded(y, gain, 128)
      List1.append(OneTime)
    #Samples ready, go for the Mid
    total=0
    for z in range(0,len(List1)):
        total += List1[z]
    if debug: print(total)
    #MidListInt.append(total/len(List1))
    return int((total/len(List1)))


#Classes:

class DisplayDriver(threading.Thread):
    def __init__(self, queue):
        threading.Thread.__init__(self)
        self.queue = Queue()
        self.running = False 
    
    def run(self):   
        if debug : a = datetime.datetime.now()
        self.running = True
        if debug : print("init display")
        #RST = 24    
        #disp = Adafruit_SSD1306.SSD1306_128_64(rst=RST)
        #disp.begin()
        #disp.clear()
        #disp.display()
        # Load default font.
        font = ImageFont.load_default()
        font2 = ImageFont.truetype("ARIALN.TTF", 20)
        #device = ssd1306(port=0, address=0x3C)
        device = sh1106(port=0, address=0x3C)
        # Create image buffer.
        width = device.width
        height = device.height
        #image = Image.new('1', (width, height))
        # Get drawing object to draw on image.
        #draw = ImageDraw.Draw(image)
        rowhight = 12
        input       = []
        inputold    = [0,0,0,0,0,0]
        #input = [126,127,128,129,10,5]  # 1=1 2=2 3=1,2 4=3 dvs knapp 3; 5 => knapp 1 och 3
        if debug : b = datetime.datetime.now()
        if debug : delta = b - a
        if debug : deltatime = delta.total_seconds()
        if debug : print("Time init=",deltatime)
        #showimage(image)
        #print("displaying startimageobject on display")
        #disp.image(image)
        #disp.display()

        #----------------------------------------------------------------------------------------------------------

        while (self.running):
            if debug : a = datetime.datetime.now()
            #if debug : print("starting loop")
            repeat = 1
            # grabing the last value in queue that differs from inputold and save in variable input
            while (repeat == 1):
                inputtemp = queue.get()
                #if debug : print("comparing inputtemp and inputold: ",inputtemp,inputold)        
                if inputtemp != inputold : 
                    #if debug : print("differs, saving in variable input")
                    input = inputtemp
                #if debug : print(queue.qsize())
                if ((queue.qsize() == 0) and (input != inputold)) : repeat = 0
            if debug : print("leaving queuehandeling, input determed to: ",input)    
            if inputold != inputtemp : input = inputtemp
            if input == [] : break  # to asure to get out of loop at program termination
            #if debug : b = datetime.datetime.now()
            #if debug : delta = b - a
            #if debug : deltatime = delta.total_seconds()
            #if debug : print("queuetime: ",deltatime)
            if debug : a = datetime.datetime.now()
            with canvas(device) as draw:
                if input[0] == 999:
                    if debug : print("text is: ", input[1])
                    twidth,theight = font2.getsize(input[1])
                    draw.text(((width-twidth)/2,(height-theight)/2), input[1],  font=font2, fill=255)
                    time.sleep(2)
                else:
                    # Draw it all
                    i = 0
                    while i != 3:
                        draw.line((0,i*rowhight,width,i*rowhight),fill=255)
                        i += 1
                    i = 0
                    #print("drawing static lines")
                    while i != 8:
                        draw.line((i*(width//8),0,i*(width//8),2*rowhight),fill=255)
                        i += 1
                    draw.line((127,0,127,2*rowhight),fill=255)
                    draw.line((0,64,64,2*rowhight),fill=255)
                    draw.line((64,2*rowhight,64,64),fill=255)
                    draw.line((64,64,127,2*rowhight),fill=255)
                    # from left, from top
        
                    #print("filling boxes")
                    i = 1
                    while i < 16:
                        ButtonNo = i//2+1
                        text = str(ButtonNo)
                        twidth,theight = font.getsize("88")
                        x0=(i*(width//16))-(twidth//2)
                        y0=(rowhight-theight)//2+1
                        x1=(i*(width//16))-(twidth//2)+twidth
                        y1=(rowhight-theight)//2+1+theight-1
                        if (input[4] & (1 << (ButtonNo-1))) > 0 :
                            draw.rectangle([x0,y0,x1,y1],fill=255, outline=255)
                        else:
                            draw.text(((i*(width//16))-(twidth//2), (rowhight-theight)//2+1),text,  font=font, fill=255)
                        i +=2
                    i = 1
                    while i < 16:
                        ButtonNo = i//2+9
                        text = str(ButtonNo)
                        twidth,theight = font.getsize("88")
                        x0=(i*(width//16))-(twidth//2)
                        y0=(rowhight-theight)//2++rowhight+1
                        x1=(i*(width//16))-(twidth//2)+twidth
                        y1=(rowhight-theight)//2++rowhight+1+theight-1
                        if (input[5] & (1 << (ButtonNo-9))) > 0 :
                            draw.rectangle([x0,y0,x1,y1],fill=255, outline=255)
                        else:
                            draw.text(((i*(width//16))-(twidth//2), (rowhight-theight)//2+rowhight+1),text,  font=font, fill=255)
                        i +=2
                    draw.text((2,(2*rowhight)), str(input[0]), font=font2, fill=255)
                    draw.text((29,(64-(2*rowhight))//2+(2*rowhight)), str(input[1]), font=font2, fill=255)
                    draw.text((66,(2*rowhight)), str(input[2]), font=font2, fill=255)
                    draw.text((93,(64-(2*rowhight))//2+(2*rowhight)), str(input[3]), font=font2, fill=255)

                    # Display image.
                    if debug : b = datetime.datetime.now()
                    if debug : delta = b - a
                    if debug : deltatime = delta.total_seconds()
                    if debug : print("looptime excl display: ",deltatime)
            
                    if debug : a = datetime.datetime.now()
                    #self.queue.task_done()
            if debug : b = datetime.datetime.now()
            if debug : delta = b - a
            if debug : deltatime = delta.total_seconds()
            if debug : print("displaytime: ",deltatime)
            inputold = input
        self.running = False

    def stop(self):
        self.running = False


#INIT !

print("Initializing port")
port = serial.Serial("/dev/ttyAMA0", 115200)
print("Initializing AD")
adc = ADS1x15(ic=0x00)
gain = 4096  # +/- 4.096V
samples = 1
print("Starting displaydriver")
oledD = DisplayDriver(queue) 
queue.put([999,"Starting"])
print("Starting threads")
oledD.start()

print("Initializing pin")
pin_base = 65       # lowest available starting number is 65  
i2c_addr = 0x20     # A0, A1, A2 pins all wired to GND  
wiringpi.wiringPiSetup()                    # initialise wiringpi  
wiringpi.mcp23017Setup(pin_base,i2c_addr)   # set up the pins and i2c address  
for x in range(0,16):
  wiringpi.pinMode(pin_base+x, 0)           # set pin as input
  wiringpi.pullUpDnControl(pin_base+x, 2)   # set internal pull-up 

print("Initializing configfile")
config = ConfigParser.RawConfigParser()
configFile='./config.cfg'
if os.path.isfile(configFile) and os.access(configFile, os.R_OK):
    print("configFile exists and is readable, getting conf:")
    config.read(configFile)
    hasConfigFile = True
    config.read(configFile)
    LowCenter = []             # Initialize readings
    HighCenter = []
    LowEdge = []
    HighEdge = []
    LowCenter.append(config.getint('LC', 'LC0'))
    LowCenter.append(config.getint('LC', 'LC1'))
    LowCenter.append(config.getint('LC', 'LC2'))
    LowCenter.append(config.getint('LC', 'LC3'))
    HighCenter.append(config.getint('HC', 'HC0'))
    HighCenter.append(config.getint('HC', 'HC1'))
    HighCenter.append(config.getint('HC', 'HC2'))
    HighCenter.append(config.getint('HC', 'HC3'))
    LowEdge.append(config.getint('LE', 'LE0'))
    LowEdge.append(config.getint('LE', 'LE1'))
    LowEdge.append(config.getint('LE', 'LE2'))
    LowEdge.append(config.getint('LE', 'LE3'))
    HighEdge.append(config.getint('HE', 'HE0'))
    HighEdge.append(config.getint('HE', 'HE1'))
    HighEdge.append(config.getint('HE', 'HE2'))
    HighEdge.append(config.getint('HE', 'HE3'))  
    print(LowCenter)
    print(HighCenter)
    print(LowEdge)
    print(HighEdge)
else:
    print("Either configFile is missing or is not readable")
    hasConfigFile = False


#Starating !
print("Starting")

reconfigure = False
if hasConfigFile:
  print("Press start to Reconfigure, other to continue")
  while True: 
    lsb,msb=scanButtons()
    if ((lsb>0) or (msb>0)): break
  if ((lsb == 128) and (msb == 0)): 
    reconfigure = True
  else: 
    reconfigure = False

time.sleep(0.1)
while True:                             # Wait for button release
    lsb,msb=scanButtons()  
    if ((lsb==0) or (msb==0)): break    # if no button pressed, continue
    time.sleep(0.01)

time.sleep(0.1)
if ((reconfigure == True) or (hasConfigFile == False)): # if startpressed or no configfile 
  print("Calibrating Center, press button to continue")
  LowCenter = [-1,-1,-1,-1]             # Initialize readings
  HighCenter = [-1,-1,-1,-1]
  LowEdge = [-1,-1,-1,-1]
  HighEdge = [-1,-1,-1,-1]

  while True:
    lsb,msb=scanButtons() 
    if ((lsb>0) or (msb>0)): break #Wile no button is pressed
    for y in range(0,4):
      snitt = GetAD(y)
      if LowCenter[y] == -1:
        LowCenter[y] = snitt
      elif LowCenter[y] > snitt:
        LowCenter[y] = snitt
      if HighCenter[y] == -1:
        HighCenter[y] = snitt
      elif HighCenter[y] < snitt:
        HighCenter[y] = snitt
  print("Center:")
  print(LowCenter)
  print(HighCenter)
  
  time.sleep(0.1)
  while True:                           # Wait for button release
    lsb,msb=scanButtons()  
    if ((lsb==0) or (msb==0)): break    # if no button pressed, continue
    time.sleep(0.01)

  time.sleep(0.1)
  print("Calibrating Edge, press button to continue")
  while True:
    lsb,msb=scanButtons()
    if ((lsb>0) or (msb>0)): break #Wile no button is pressed
    for y in range(0,4):
      snitt = GetAD(y)
      if LowEdge[y] == -1:
        LowEdge[y] = snitt
      elif LowEdge[y] > snitt:
        LowEdge[y] = snitt
      if HighEdge[y] == -1:
        HighEdge[y] = snitt
      elif HighEdge[y] < snitt:
        HighEdge[y] = snitt
  print("Edge:")
  print(LowEdge)
  print(HighEdge)
  
  time.sleep(0.1)
  while True:                           # Wait for button release
    lsb,msb=scanButtons()  
    if ((lsb==0) or (msb==0)): break    # if no button pressed, continue
    time.sleep(0.01)
  print("Writing down config file")
  if (not(config.has_section("LC"))): config.add_section("LC")
  config.set('LC', 'LC0',LowCenter[0])
  config.set('LC', 'LC1',LowCenter[1])
  config.set('LC', 'LC2',LowCenter[2])
  config.set('LC', 'LC3',LowCenter[3])
  if (not(config.has_section("HC"))): config.add_section("HC")
  config.set('HC', 'HC0',HighCenter[0])
  config.set('HC', 'HC1',HighCenter[1])
  config.set('HC', 'HC2',HighCenter[2])
  config.set('HC', 'HC3',HighCenter[3])
  if (not(config.has_section("LE"))): config.add_section("LE")
  config.set('LE', 'LE0',LowEdge[0])
  config.set('LE', 'LE1',LowEdge[1])
  config.set('LE', 'LE2',LowEdge[2])
  config.set('LE', 'LE3',LowEdge[3])
  if (not(config.has_section("HE"))): config.add_section("HE")
  config.set('HE', 'HE0',HighEdge[0])
  config.set('HE', 'HE1',HighEdge[1])
  config.set('HE', 'HE2',HighEdge[2])
  config.set('HE', 'HE3',HighEdge[3])
  with open(configFile, 'wb') as configfiler:
    config.write(configfiler)
else:
  print("Using config file")



print("Ready to read:")
print(LowCenter)
print(HighCenter)
print(LowEdge)
print(HighEdge)

while True: #Huvudsnurra
  MidListInt = []
  MidListByte = []
  time_start = time.time()
  for y in range(0,4):
    if debug: print(y)
    snitt = GetAD(y)
    interpol = int(round(interp(snitt,[LowEdge[y],LowCenter[y],HighCenter[y],HighEdge[y]],[0,126,127,255])))
    if debug: print(interpol)
    MidListByte.append(interpol)
  if debug: print(MidListByte)
  #Och nu, read from GPIO-pin
  lsb,msb=scanButtons()
  MidListByte.append(lsb)
  MidListByte.append(msb)
  #checksum
  if debug: print("checksum")
  check = (170) ^ int(MidListByte[0])
  if debug: print(check)
  for x in range(1,6):
    check = (check) ^ int(MidListByte[x])
    if debug: print(check)
  MidListByte.append(int(check))
  #print(time.time()-time_start)
  if debug: print((MidListByte))
  #check = int(MidListByte[6]) ^ int(MidListByte[5])
  #for x in range(4,-1,-1):
  #  check = int(MidListByte[x]) ^ (check)
  #if debug: print(check)
  print(MidListByte)
  port.write((chr(170)+chr(170)+chr(int(MidListByte[0]))+chr(int(MidListByte[1]))
    +chr(int(MidListByte[2]))+chr(int(MidListByte[3]))+chr(int(MidListByte[4]))
    +chr(int(MidListByte[5]))+chr(int(MidListByte[6]))))

  queue.put(MidListByte)


