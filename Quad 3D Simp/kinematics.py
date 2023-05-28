
import numpy as np
from numpy.linalg import inv, norm
from numpy import array, asarray, matrix
from math import *
import matplotlib.pyplot as plt
from util import RotMatrix3D, point_to_rad
from mpl_toolkits.mplot3d.art3d import Poly3DCollection



class kinematics():
    
    def __init__(self):
        
        # note: leg IDs
        left_front = 0
        left_back  = 1
        right_front= 2
        right_back = 3
        
        self.right_legs = [right_front, right_back]
        
        self.link_1 = 0.045
        self.link_2 = 0.1115
        self.link_3 = 0.155
        self.phi = radians(90)
        
        # body dimensions
        self.length = 0.25205
        self.width = 0.105577
        self.hight = 0.0
        
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
        XYZ = asarray((inv(RotMatrix3D(rot,is_radians)) * \
            ((array(xyz) + self.leg_origins[legID,:] - array(center_offset)).transpose())).transpose())
        
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
        
        # CALCULATE THE COORDINATES OF THE JOINTS FOR VISUALIZATION
        j1 = np.array([0,0,0])
        
        # calculate joint 3
        j3_ = np.reshape(np.array([self.link_2*cos(theta_2),0, self.link_2*sin(theta_2)]),[3,1])
        j3 = np.asarray(j2 + np.reshape(np.linalg.inv(rot_mtx)*j3_, [1,3])).flatten()
        
        # calculate joint 4
        j4_ = j3_ + np.reshape(np.array([self.link_3*cos(theta_2+theta_3),0, self.link_3*sin(theta_2+theta_3)]), [3,1])
        j4 = np.asarray(j2 + np.reshape(np.linalg.inv(rot_mtx)*j4_, [1,3])).flatten()

        
        
        # modify angles to match robot's configuration (i.e., adding offsets)
        angles = self.angle_corrector(angles=[theta_1, theta_2, theta_3], is_right=is_right)
        # print(degrees(angles[0]))
        return [angles[0], angles[1], angles[2], j1, j2, j3, j4]
    
    
    def base_pose(self, rot=[0,0,0], is_radians=True, center_offset=[0,0,0]):
        
        # offset due to non-centered axes of rotation
        offset = RotMatrix3D(rot, is_radians) * \
            (matrix(center_offset).transpose()) - matrix(center_offset).transpose()
        
        # rotate the base around the center of rotation (if there is no offset, then the center of 
        # rotation will be at the center of the robot)
        rotated_base = RotMatrix3D(rot, is_radians) * self.leg_origins.transpose() - offset
        return rotated_base.transpose()
       
    # get coordinates of leg joints relative to j1
    def leg_pose(self, xyz, rot, legID, is_radians, center_offset=[0,0,0]):
        
        # get the coordinates of each joints relative to the leg's origin
        pose_relative = self.leg_IK(xyz, rot, legID, is_radians, center_offset)[3:]
        
        # adjust the coordinates according to the robot's orientation (roll, pitch, yaw)
        pose_true = RotMatrix3D(rot,is_radians) * (array(pose_relative).transpose())
        return pose_true.transpose()
    
    # plot rectangular base where each corner represents the origin of leg
    def plot_base(self, ax, rot=[0,0,0], is_radians=True, center_offset=[0,0,0]):

        p = self.base_pose(rot, is_radians, center_offset).transpose()

        print(p)

        
    
    # Get the minimum and maximum coordinates for each axis
        x_min, x_max = np.min(p[0]), np.max(p[0])
        y_min, y_max = np.min(p[1]), np.max(p[1])
        z_min, z_max = np.min(p[2]), np.max(p[2])
    
    # Define the coordinates of the vertices of the cuboid
        vertices = np.array([
        [x_min, y_min, z_min],
        [x_min, y_max, z_min],
        [x_max, y_max, z_min],
        [x_max, y_min, z_min],
        [x_min, y_min, z_max],
        [x_min, y_max, z_max],
        [x_max, y_max, z_max],
        [x_max, y_min, z_max]
        ])
    
    # Define the indices of the cuboid's faces
        faces = np.array([
        [0, 1, 2, 3],
        [4, 5, 6, 7],
        [0, 1, 5, 4],
        [1, 2, 6, 5],
        [2, 3, 7, 6],
        [3, 0, 4, 7]
         ])
        

        
       

        
        cuboid = Poly3DCollection([vertices[face] for face in faces], alpha=1, edgecolor='black')
        shadow = Poly3DCollection([vertices[face] for face in faces], alpha=0.3, edgecolor='black', facecolor='gray')
        

        cuboid.set_facecolor('White')
    
        cuboid.set_3d_properties
        ax.add_collection3d(cuboid)
        ax.add_collection3d(shadow)

        return ax
    
    
    
    # Plot the cuboid
    ''' for face in faces:
         ax.plot3D(vertices[face, 0], vertices[face, 1], vertices[face, 2],'r' )#'white', alpha=0)
    
        return ax '''
       
    # plot leg 
    def plot_leg(self, ax, xyz, rot=[0,0,0], legID=0, is_radians=True, center_offset=[0,0,0]):
        # get coordinates
        p = ((self.leg_pose(xyz, rot, legID, is_radians, center_offset) \
                + self.base_pose(rot,is_radians,center_offset)[legID]).transpose())
        # plot coordinates
        line_widths = [1.5, 3.0, 4.5]  # Example line widths
        for i, lw in enumerate(line_widths):
         ax.plot3D(asarray(p[0,:]).flatten(), asarray(p[1,:]).flatten(), asarray(p[2,:]).flatten(), 'black', linewidth=lw,)

        ax.plot3D(asarray(p[0,:]).flatten(), asarray(p[1,:]).flatten(), asarray(p[2,:]).flatten(), 'b')
        
        
        end_effector_x = p[0, -1]
        end_effector_y = p[1, -1]
        end_effector_z = p[2, -1]

        

        knee_joint_x = p[0, -2]
        knee_joint_y = p[1, -2]
        knee_joint_z = p[2, -2]

       
        hip_joint_x, hip_joint_y, hip_joint_z = p[:, -3]
        main_joint_x, main_joint_y, main_joint_z = p[:, -4]


        
     
        # Plot a round ball at the end effector coordinates
        ax.scatter(knee_joint_x, knee_joint_y, knee_joint_z, color='red', s=3)
        ax.scatter(end_effector_x, end_effector_y, end_effector_z, color='green', s=30)
        ax.scatter(hip_joint_x, hip_joint_y, hip_joint_z, color='red', s=3)
        ax.scatter(main_joint_x, main_joint_y, main_joint_z, color='red', s=14, alpha=0.3, edgecolor='white')
        ax.scatter(main_joint_x, main_joint_y, main_joint_z, color='grey', s=14, alpha=0.3)





        return

    def plot_robot(self, xyz, rot=[0,0,0], leg_N=4, is_radians=True, limit=0.250, center_offset=[0,0,0]):
    
        ax = self.ax_view(limit)  # set the view
        self.plot_base(ax,rot, is_radians, center_offset)  # plot base

        # plot legs
        for leg in range(leg_N):
            self.plot_leg(ax,xyz[leg],rot,leg, is_radians, center_offset) 
        
        # show figure
        plt.show()
        return

        
    # TO-DO : modify this function depending on your robot's configuration
    # adjusting angle for specific configurations of motors, incl. orientation
    # this will vary for each robot (possibly for each leg as well)
    def angle_corrector(self, angles=[0,0,0], is_right=True):
        angles[1] -= 1.5*pi; # add offset 
        
        if is_right:
            theta_1 = angles[0] - pi
            theta_2 = angles[1] + 45*pi/180 # 45 degrees initial offset
        else: 
            if angles[0] > pi:  
                theta_1 = angles[0] - 2*pi
            else: theta_1 = angles[0]
            
            theta_2 = -angles[1] - 45*pi/180
        
        theta_3 = -angles[2] + 45*pi/180
        return [theta_1, theta_2, theta_3]
        
    # set view  
    @staticmethod
    def ax_view(limit):
        ax = plt.axes(projection="3d")
        ax.set_facecolor('lightblue')
        ax.light_sources = [(1, 1, 1)] 

        light_source1 = (1, 1, 1)  # Light source 1 position
        light_source2 = (-1, -1, 1)  # Light source 2 position
        ax.light_sources = [light_source1, light_source2]


        ax.set_xlim(-limit, limit)
        ax.set_ylim(-limit, limit)
        ax.set_zlim(-limit, limit)
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        return ax
    




