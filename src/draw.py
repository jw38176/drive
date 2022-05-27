from cmath import pi
import serial
import math
from turtle import *

rover = serial.Serial('COM4', 9600, timeout = 3)
print(rover.name)

data = rover.readline()
data = rover.readline()


while True:
    data = rover.readline().strip().decode('utf-8')
    print(data)
    goto(data)
    #d = data.split(",")
    #total_x = float(d[0])
    #total_y = float(d[1])

    #goto(total_x, total_y)
    
    #headings = (asin((distance_x/100)/13.5)*180/pi)

    #setheading(90+headings)
    #print(headings)

rover.close() 
