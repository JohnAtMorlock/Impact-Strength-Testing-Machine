# This is firmware for a impactt strength testing machine. It uses a 10K potentiometer to observe the arc transversed
# by a swining hammer. The microcontroller is a Raspberry Pico running micropython. The analog output of the 10K pot is dumped into the ADC on pin 28. The OLED uses I2C on pins 0 and 1. 

#Important note: The pot must be set up so the 90 degree position, which is the resting position of the hammer, is half way through
# its range of motion. a 10K pot has 300 degrees of motion. 

# Programing Instructions: 
# First flash the raspberry pi with micropython. Then upload the ssd1306.py file to the raspberry pico using Pico-Go on Visual Studios Code. 
# Finally, upload the main.py as well using the same workflow (Pico-Go on Visual Studios Code)

#When first turned on, the machine will spend 2 seconds auto-calibrating its self to find the 90 degree resting postion. 

from machine import Pin, I2C
from ssd1306 import SSD1306_I2C
import time
import math

# mass of the end of the hammer. This is usually denoted by the seller. Here a 16 0z hammer is used. The value bellow 
# must be in kilograms!
global hammer_Mass
hammer_Mass = 0.4536
# the distance between the point of rotation and the middle of the hammer's head; must be in meters! 
global hammer_Length
hammer_Length = 0.28
# the angle relative to the horizontal at which the impact test starts logging data
global critical_Angle
critical_Angle = -25
# determined expirementally using the "calculate_Efficiency" function. It is the loss from friction on the bearings
global energy_Loss_Constant
energy_Loss_Constant = 0.94

#I2C setup for the OLED display 
i2c=I2C(0,sda=Pin(0), scl=Pin(1), freq=400000)
oled = SSD1306_I2C(128, 64, i2c)
# other pins and the internal LED
LED_Pin = Pin(25, Pin.OUT)
hammer_Pot_Pin = machine.ADC(28)

global degrees
global offset
# determined by the code later during initial calibration when the machine is turned on
offset = 0
data_Points = []

# branding 
oled.text("MORLOCK", 30, 0)
oled.show()

# creating a map function by hand, because python for some reason does not have one
def map(x, in_min, in_max, out_min, out_max):
    return int((x-in_min) * (out_max-out_min) / (in_max-in_min) + out_min)

# function to convert the 10K pot and ADC readings to degrees
def readDegrees(offset):
    global degrees
    hammer_Pot_Value = hammer_Pot_Pin.read_u16()
    # analog reads on a pico range between 65535 for 3.3 volts to 0 to 0 volts.
    degrees = -(map(int(hammer_Pot_Value), 0, 65535, 0, 300)) + offset + 90

# find the resting point of the hammer, which is defined as 90 degrres relative to the horizon
def origin_Calibration(trials, delay):
    # calibrate to origin point
    # create empty list
    calibration_Readings = []
    # wait assigned time for calibration
    time.sleep(delay)
    # fill up list with data points
    for i in range(trials):
        hammer_Pot_Value = hammer_Pot_Pin.read_u16()
        degrees = map(int(hammer_Pot_Value), 0, 65535, 0, 300)
        # start recording data for calibration
        calibration_Readings.append(degrees)
        time.sleep(0.01)
    # average data points in list
    global orgin_Offset_Angle
    orgin_Offset_Angle = sum(calibration_Readings)/trials

# function used to display degrees: Good for debugging
def displayDegrees(value):
    oled.fill(0)
    oled.text("Morlock", 30, 0)
    oled.text("Degrees", 0, 40)
    oled.text(str(value), 60, 40)
    oled.show()

# used to do the actual impact test. It finds the range of the degrees meased (min and max) and used that to calcuated the energy of the hammer before
# and after impact. The formula used is mass*gravity*height = joules of energy 
def impact_Test_Begin(): 
    # the starting and ending heights of the hammer. 
    global starting_Height 
    global ending_Height
    # do nothing while the hammer is being pulled back. Also start recording data
    while degrees < 90:
        readDegrees(orgin_Offset_Angle)
        displayDegrees(str(int(degrees)))
        time.sleep(0.01)
        data_Points.append(degrees)
    while degrees >= 90:
        readDegrees(orgin_Offset_Angle)
        displayDegrees(str(int(degrees)))
        time.sleep(0.01)
        data_Points.append(degrees)
    # wait forthe hammer to get back to about the starting point on the down swing, and then find the maximum point
    ending_Angle = max(data_Points)
    ending_Angle = ending_Angle - 90
    if ending_Angle >= 90:
        ending_Angle = ending_Angle - 90
        ending_Height = hammer_Length*(1 - math.cos(math.radians(ending_Angle))) + hammer_Length
    else:
        ending_Height = hammer_Length*(1 - math.cos(math.radians(ending_Angle))) ####################
    # do the calculations for the minimum/ending point 
    starting_Angle = min(data_Points)
    if starting_Angle < 0: # this code is needed to compensate for negative angles relative to the horizon
        starting_Angle = abs(starting_Angle)
        starting_Height = hammer_Length*(1 - math.cos(math.radians(starting_Angle))) + hammer_Length ###############
    else:
        starting_Height = hammer_Length*(1-math.cos(math.radians(starting_Angle)))
        return()

# a function used to calculate the efficiency of the rig. The hammer is pulled back to the limit and released. No target is used, so the only energyy 
# lost is due to friction. This function is uncommented bellow by default. 
def calculate_Efficiency(starting_Height, ending_Height, hammer_Mass):
    global calculated_Efficiency
    calculated_Efficiency = (ending_Height/starting_Height)
    oled.fill(0)
    oled.text("Morlock", 30, 0)
    oled.text("Eff ", 0, 40)
    oled.text(str(calculated_Efficiency), 60, 40)
    print("start: " + str(starting_Height))
    print("ending: " + str(ending_Height))
    oled.show()
    
    # used to calculated how much energy was absorpted by the target using the mass*gravity* height formula
def energy_Absorption_Calculation(starting_Height, ending_Height, energy_Loss_Constant, hammer_Mass): 
    global impact_Strength
    impact_Strength = (starting_Height-(ending_Height/energy_Loss_Constant))*hammer_Mass*9.81
    oled.fill(0)
    oled.text("Morlock", 30, 0)
    oled.text("energy", 0, 40)
    oled.text(str(round(impact_Strength, 3)), 70, 40)
    oled.show()

# void setup
origin_Calibration(100, 1)
# void loop
while True:
    # clear any old data stored
    data_Points.clear()
    readDegrees(orgin_Offset_Angle)
    #displayDegrees(degrees)
    # if the hammer is pulled back, then begin testing
    if degrees < critical_Angle: 
        impact_Test_Begin()
        #calculate_Efficiency(starting_Height, ending_Height, hammer_Mass)
        energy_Absorption_Calculation(starting_Height, ending_Height, energy_Loss_Constant, hammer_Mass)
    # turn on the LED for show and then do a delay
    LED_Pin.toggle()
    time.sleep(0.01)

