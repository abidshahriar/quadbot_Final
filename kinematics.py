import numpy as np
from numpy.linalg import inv, norm
from numpy import array, asarray, matrix
from math import *
from math import atan, pi, radians, cos, sin


import serial

ser = serial.Serial('/dev/ttyACM0', 9600)



def point_to_rad(p1, p2): # converts 2D cartesian points to polar angles in range 0 - 2pi
        
    if (p1 > 0 and p2 >= 0): return atan(p2/(p1))
    elif (p1 == 0 and p2 >= 0): return pi/2
    elif (p1 < 0 and p2 >= 0): return -abs(atan(p2/p1)) + pi
    elif (p1 < 0 and p2 < 0): return atan(p2/p1) + pi
    elif (p1 > 0 and p2 < 0): return -abs(atan(p2/p1)) + 2*pi
    elif (p1 == 0 and p2 < 0): return pi * 3/2
    elif (p1 == 0 and p2 == 0): return pi * 3/2 # edge case
    
def RotMatrix3D(rotation=[0,0,0],is_radians=True, order='xyz'):
    
    roll, pitch, yaw = rotation[0], rotation[1], rotation[2]

    # convert to radians is the input is in degrees
    if not is_radians: 
        roll = radians(roll)
        pitch = radians(pitch)
        yaw = radians(yaw)
    
    # rotation matrix about each axis
    rotX = np.matrix([[1, 0, 0], [0, cos(roll), -sin(roll)], [0, sin(roll), cos(roll)]])
    rotY = np.matrix([[cos(pitch), 0, sin(pitch)], [0, 1, 0], [-sin(pitch), 0, cos(pitch)]])
    rotZ = np.matrix([[cos(yaw), -sin(yaw), 0], [sin(yaw), cos(yaw), 0], [0, 0, 1]])
    
    # rotation matrix order (default: pitch -> roll -> yaw)
    if order == 'xyz': rotationMatrix = rotZ * rotY * rotX
    elif order == 'xzy': rotationMatrix = rotY * rotZ * rotX
    elif order == 'yxz': rotationMatrix = rotZ * rotX * rotY
    elif order == 'yzx': rotationMatrix = rotX * rotZ * rotY
    elif order == 'zxy': rotationMatrix = rotY * rotX * rotZ
    elif order == 'zyx': rotationMatrix = rotX * rotY * rotZ
    
    return rotationMatrix # roll pitch and yaw rotation 


class kinematics():

    def __init__(self):
        
        # note: leg IDs
        left_front = 0
        left_back  = 1
        right_front= 2
        right_back = 3
        
        self.right_legs = [right_front, right_back]
        
        self.link_1 = 0.044
        self.link_2 = 0.28
        self.link_3 = 0.277
        self.phi = radians(90)
        
        # body dimensions
        self.length = 0.6
        self.width = 0.3
        self.hight = 0.1

        # leg origins (left_f, left_b, right_b, right_f), i.e., the coordinate of j1
        self.leg_origins = np.matrix([[self.length/2, self.width/2, 0],
                          [-self.length/2, self.width/2, 0],
                          [-self.length/2, -self.width/2, 0],
                          [self.length/2, -self.width/2, 0],
                          [self.length/2, self.width/2, 0]])
        
    # this method adjust inputs to the IK calculator by adding rotation and 
    # offset of that rotation from the center of the robot
    def leg_IK(self, xyz, rot = [0,0,0], legID=0, is_radians=True, center_offset=[0,0,0]):
        
        # check is the leg is from the right side 
        is_right = (legID in self.right_legs)
        
        # add offset of each leg from the axis of rotation
        XYZ = asarray((inv(RotMatrix3D(rot,is_radians))*((array(xyz) + self.leg_origins[legID,:] - array(center_offset)).transpose())).transpose())
        
        # subtract the offset between the leg and the center of rotation 
        # so that the resultant coordiante is relative to the origin (j1) of the leg
        xyz_ = asarray(XYZ - self.leg_origins[legID,:] + array(center_offset)).flatten()

        # calculate the angles and coordinates of the leg relative to the origin of the leg
        return self.leg_IK_calc(xyz_, is_right)
    





    # IK calculator
    def leg_IK_calc(self, xyz, is_right=False): 

        x, y, z = xyz[0], xyz[1], xyz[2]    # unpack coordinates
        
        # length of vector projected on the YZ plane. equiv. to len_A = sqrt(y**2 + z**2)
        len_A = norm([0,y,z])   
        
        # a_1 : angle from the positive y-axis to the end-effector (0 <= a_1 < 2pi)
        # a_2 : angle bewtween len_A and leg's projection line on YZ plane
        # a_3 : angle between link1 and length len_A
        a_1 = point_to_rad(y,z)                     
        a_2 = asin(sin(self.phi)*self.link_1/len_A) 
        a_3 = pi - a_2 - self.phi                   
        
        # angle of link1 about the x-axis 
        if is_right: theta_1 = a_1 - a_3
        else: 
            theta_1 = a_1 + a_3
            if theta_1 >= 2*pi: theta_1 -= 2*pi
        
        j2 = array([0,self.link_1*cos(theta_1),self.link_1*sin(theta_1)])
        j4 = array(xyz)
        j4_2_vec = j4 - j2 # vector from j2 to j4
        
        if is_right: R = theta_1 - self.phi - pi/2
        else: R = theta_1 + self.phi - pi/2
        
        # create rotation matrix to work on a new 2D plane (XZ_)
        rot_mtx = RotMatrix3D([-R,0,0],is_radians=True)
        j4_2_vec_ = rot_mtx * (np.reshape(j4_2_vec,[3,1]))
        
        # xyz in the rotated coordinate system + offset due to link_1 removed
        x_, y_, z_ = j4_2_vec_[0], j4_2_vec_[1], j4_2_vec_[2]
        
        len_B = norm([x_, z_]) # norm(j4-j2)
        
        # handling mathematically invalid input, i.e., point too far away to reach
        if len_B >= (self.link_2 + self.link_3): 
            len_B = (self.link_2 + self.link_3) * 0.99999
            # self.node.get_logger().warn('target coordinate: [%f %f %f] too far away' % (x, y, z))
            print('target coordinate: [%f %f %f] too far away' % (x, y, z))
        
        # b_1 : angle between +ve x-axis and len_B (0 <= b_1 < 2pi)
        # b_2 : angle between len_B and link_2
        # b_3 : angle between link_2 and link_3
        b_1 = point_to_rad(x_, z_)  
        b_2 = acos((self.link_2**2 + len_B**2 - self.link_3**2) / (2 * self.link_2 * len_B)) 
        b_3 = acos((self.link_2**2 + self.link_3**2 - len_B**2) / (2 * self.link_2 * self.link_3))  
        
        # assuming theta_2 = 0 when the leg is pointing down (i.e., 270 degrees offset from the +ve x-axis)
        theta_2 = b_1 - b_2    
        theta_3 = pi - b_3

        # modify angles to match robot's configuration (i.e., adding offsets)
        angles = self.angle_corrector(angles=[theta_1, theta_2, theta_3], is_right=is_right)
        print(degrees(angles[0]))
        print('t1',(degrees(angles[0])));
        print('t2',(degrees(angles[1])));
        print('t3',(degrees(angles[2])));
        
        

        ser.write((f"{degrees(angles[0])},{degrees(angles[1])},{degrees(angles[2])}\n".encode()))

        print('-----------')
        return theta_1, theta_2, theta_3
    
  
    def angle_corrector(self, angles=[0,0,0], is_right=True):
        angles[1] -= 1.5*pi; # add offset  //2nd joint// //nicher dike lomba rakhar jnno 2nd joint ta, negatve karon main angle t2 o o negative ashe karon nicher dike -z axis //
        ##angles[2] -= 1.5*pi;
        
        
        #right side er leg gulor angle positive ashe  tai theta 2 er offset e +ve 45
        if is_right:
            theta_1 = angles[0] - pi   #1st angle , to keep it under 180
            theta_2 = angles[1] + 45*pi/180  # 45 degrees initial offset (ok)

        #left side er leg er angle gulo sob negative ashe tai negative side er leg gulor theta2  negative 45.
          
        else: 
            if angles[0] > pi:  
                theta_1 = angles[0] - 2*pi  ##ei kahini korar karon angle re sobsosmoy 180 er moodhe rakha .. oita exceed korle ore negative banano
            else: theta_1 = angles[0]       #180 er niche holi ja ache tai
            
            theta_2 = -angles[1] - 45*pi/180 # 45 degree offset (ok)
        
        theta_3 = -angles[2] + 80*pi/180 

      
        return [theta_1, theta_2, theta_3]
    
    

    



'''ghorir katar biportit dik +angle
ghorir katar dik - angle'''
