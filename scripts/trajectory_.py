#!/usr/bin/python3
import math
from re import S
import numpy as np
from numpy import ones, zeros
import rclpy
import sys, os, yaml
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory , JointTrajectoryPoint
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu

class Trajectory_publisher(Node):

    def __init__(self):        
        super().__init__('trajectory_publsiher_node')
        self.IMU_subscriber = self.create_subscription(Imu,'/Imu',self.IMU,10)      # create subscriber topic /IMU (type: Imu)          
        self.trajectory_publihser = self.create_publisher(JointTrajectory, "/joint_trajectory_position_controller/joint_trajectory" , 10)  #publish topic /joint_trajectory_position_controller/joint_trajectory (type: JointTrajectory)
        timer_period = 0.1  #sce
        self.timer = self.create_timer(timer_period, self.timer_callback)   #create timer and call function timer_callback 
        self.joints = ['joint_1','joint_2','joint_3'] # name joint to control
        self.position_x = 0.157     # offset position x axis
        self.position_y = 0.0       # offset position y axis
        self.position_z = 0.12      # offset position z axis

    def IMU(self,msg:Imu):          # function IMU (input IMU)
        h1 = 0.08                      # set length l0 = 0.08
        l1 = 0.025                     # set length l1 = 0.025
        l2 = 0.07                      # set length l2 = 0.07
        l3 = 0.04                      # set length l3 = 0.04
        time = 1                       # set time = 1
        t = 500                        # set t = 500 
        a = msg.linear_acceleration.x                   #keep linear acceleration x axis in varible a
        b = msg.linear_acceleration.y                   #keep linear acceleration y axis in varible b
        if abs(msg.linear_acceleration.x) < 2:          # if absolute value of linear acceleration x axis < 2
            a = 0                                       # set a = 0
        if abs(msg.linear_acceleration.y) < 2:          # if absolute value of linear acceleration y axis < 2
            b = 0                                       # set b = 0
 
        if abs(a)>abs(b):                               # if absolute value a > absolute value b                 
            self.position_x += (a*time)/t               # set position x axis add value of (a*time)/2
        else:                                           # if absolute value a <= absolute value b
            self.position_y += (b*time)/t               # set position y axis add value of (b*time)/2

        gramma = [-1,1]                                 # set gramma index0 = -1, index1 = 1

        #Calculate Inverse Kinematics
        r = self.position_x**2 + self.position_y**2     # set r = (position x axis)^2 + (position y axis)^2
        x3 = r-l1                                       # set x3 = r-ll
        y3 = self.position_z-h1                         # set y3 = position z axis - h1
        c3 = (x3**2+y3**2-l2**2-l3**2)/(2*l2*l3)        # set c3 = (x3^2+y3^2-l2^2-l3^2)/(2*l2*l3) 

        if r > 0.1308:              # if position > The distance the robot can go 
            print('more')               # print 'more'

        elif c3 <= 1:               # if c3 <= 1 (Calculate Inverse Kinematics)
            s3 = gramma[1]*math.sqrt(1-c3**2)      # set s3 = Square root of 1-c3^2
            theta_1 = np.arctan2(self.position_y/gramma[0]*r,self.position_x/gramma[1]*r)    # set theta_1 = arctan2(position y axis/-r,position x/r)
            theta_2 = np.arctan2(y3,x3) - np.arctan2(l3*s3,l2+l3*c3)     # set theta_2 = arctan2(y3,x3)
            theta_3 = np.arctan2(s3,c3)     #  set theta_3 = arctan2(s3,c3)

            self.q = [theta_1 ,theta_2 ,theta_3]       # set q = [theta_1 ,theta_2 ,theta_3] 

    def timer_callback(self):                           # function timer_callback       
        bazu_trajectory_msg = JointTrajectory()         # set bazu_trajectory_msg = JointTrajectory
        bazu_trajectory_msg.joint_names = self.joints   # set bazu_trajectory_msg.joint_names = self.joints  
        ## creating a point
        point = JointTrajectoryPoint()
        point.positions = self.q                        # set point.positions = q
        point.time_from_start = Duration(sec=1)         # set time durataion point
        ## adding newly created point into trajectory message
        bazu_trajectory_msg.points.append(point)        
        self.trajectory_publihser.publish(bazu_trajectory_msg)  # publish bazu_trajectory_msg
        # print(self.setpoint_position)
 
def main(args=None):
    rclpy.init(args=args)
    joint_trajectory_object = Trajectory_publisher()
    try:
        while rclpy.ok():
            rclpy.spin_once(joint_trajectory_object)
    except KeyboardInterrupt:
        print('repeater stopped cleanly')
    except BaseException:
        print('exception in repeater:', file=sys.stderr)
        raise
    finally:
        joint_trajectory_object.destroy_node()
        rclpy.shutdown() 

if __name__ == '__main__':
    main()