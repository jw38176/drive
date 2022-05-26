from cmath import pi
import serial
import math
from turtle import *

rover = serial.Serial('COM4', 9600, timeout = 3)
print(rover.name)



while True:
    data = rover.readline().strip().decode('utf-8')
    d = data.split(",")
    rightspeed = int(d[0])
    leftspeed = int(d[1])
    
    leftg = int(leftspeed/5) * "*" + int(20-leftspeed/5) * "o"
    rightg = int(20-rightspeed/5) * "o" + int(rightspeed/5) * "*"
    print(leftg + "||" + rightg)
    #print ( rightspeed + "||" + leftspeed)
    #headings = (asin((distance_x/100)/13.5)*180/pi)

    #setheading(90+headings)
    #print(headings)

rover.close() 
