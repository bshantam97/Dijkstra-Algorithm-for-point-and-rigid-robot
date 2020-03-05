import numpy as np
import cv2
import math

"""
class Point:
    def __init__(self,x,y):
        self.x = x
        self.y = y
        """



x = 250
y = 150

def check_circle(c_x,c_y, rad,rigid,clearance):
    total = rigid + clearance
    c_x = 225;  
    c_y = 150;  
    rad = 25;
    rad_2 = rad * rad
    circle = ((x - c_x) * (x - c_x) + (y - c_y) * (y - c_y))
    if circle > rad_2 + total: 
        return True
    else: 
        return False  
    

def check_ellipse(h,k,a,b,rigid,clearance):
    total = rigid + clearance
    h = 150
    k = 100
    a = 40 + total
    b = 20 + total
    p = ((math.pow((x - h), 2) // math.pow(a, 2)) + 
         (math.pow((y - k), 2) // math.pow(b, 2))) 
    if p > 1 : 
        return True 
  
    else:
        return False
    


#x1 = Point(250,150)
c_x = 225
c_y = 150
rad = 25
rigid = 0
clearance = 0
if(check_circle(c_x,c_y, rad,rigid,clearance)):
    print ("outside")
else:
    print("inside")

    

 
 
 
    

