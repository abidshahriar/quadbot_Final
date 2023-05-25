import math
import serial
import time 
import kinematics
from pyPS4Controller.controller import Controller
from math import *



# Create an instance of the kinematics class
k = kinematics.kinematics()

# Define the coordinates for a single leg
x = 0
y = 0
z = -0.557

'''range z to -8.7 to -55.7 or error would come'''

'''
ser = serial.Serial('/dev/ttyACM0', 9600)

'''


# Calculate the inverse kinematics for the leg
k.leg_IK([x,y,z], rot=[0,0,0], legID=3, is_radians=True, center_offset=[0,0,0])




class MyController(Controller):
    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)

        

        self.x = x # initialize x to zero
        self.y = y #initilize y to zero
        self.z = z #initialize z to zero
        



    def set_x(self, x,y,z):
        # Calculate the inverse kinematics for the leg using the specified value of x
        k.leg_IK([x, y, z], rot=[0,0,0], legID=3, is_radians=True, center_offset=[0,0,0])
       
        
    def on_right_arrow_press(self):
        # increase x by a step size of 0.1
        self.x += 0.01
        
        # call set_x() method to calculate the inverse kinematics and print the value of theta_3
        self.set_x(self.x,self.y,self.z)
        
        # print the current value of x
        print("Current value of x: {}".format(self.x))
    
    def on_left_arrow_press(self):
        # decrease x by a step size of 0.1
        self.x -= 0.01
        
        # call set_x() method to calculate the inverse kinematics and print the value of theta_3
        self.set_x(self.x,self.y,self.z)
        
        # print the current value of x
        print("Current value of x: {}".format(self.x))
    
 
    def on_R1_press(self):

        self.y += 0.01
        self.set_x(self.x,self.y,self.z)

        print("Current value of y: {}".format(self.y))

    def on_L1_press(self):
        self.y -= 0.01
        self.set_x(self.x,self.y,self.z)

        print("Current value of y: {}".format(self.y))

    def on_up_arrow_press(self):
        self.z += 0.01
        self.set_x(self.x,self.y,self.z)  
        
        print("Current value of z: {}".format(self.z))

    def on_down_arrow_press(self):
        self.z -= 0.01
        self.set_x(self.x,self.y,self.z)  
        
        print("Current value of z: {}".format(self.z))


    

controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
# you can start listening before controller is paired, as long as you pair it within the timeout window
controller.listen(timeout=60)
