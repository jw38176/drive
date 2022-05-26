from cmath import pi
import serial
from math import acos, asin, atan
from turtle import *
rover = serial.Serial('COM4', 115200, timeout = 3)
print(rover.name)

setheading(90)

while True:
    data = rover.readline().strip().decode('utf-8')
    rover.flush()
    d = data.split(",")
    distance_x = - float(d[0])*100
    distance_y = - float(d[1])*100
    goto(distance_x, distance_y)

    headings = (asin((distance_x/100)/13.5)*180/pi)

    setheading(90+headings)
    print(headings)

rover.close() 
