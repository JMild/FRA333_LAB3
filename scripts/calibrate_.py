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
from sensor_msgs.msg import Imu
# from geometry_msgs.msg import Twist
# from geometry_msgs.msg import Vector3

class Trajectory_publisher(Node):

    def __init__(self):
        super().__init__('trajectory_publsiher_node')
        self.IMU_subscriber = self.create_subscription(Imu,"/Imu_arduino",self.IMU_callback,10)
        self.publisher_ = self.create_publisher(Imu,'/Imu', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.msg = Imu()
        self.ca = []
        self.cl = []

        self.i = 0
        self.data = 10
        self.a_ax = np.array([[]])
        self.a_ay = np.array([[]])
        self.a_az = np.array([[]])
        self.l_lx = np.array([[]])
        self.l_ly = np.array([[]])
        self.l_lz = np.array([[]])
        self.cov_axx = []
        self.cov_axy = []
        self.cov_axz = []
        self.cov_ayy = []
        self.cov_ayz = []
        self.cov_azz = []
        self.cov_lxx = []
        self.cov_lxy = []
        self.cov_lxz = []
        self.cov_lyy = []
        self.cov_lyz = []
        self.cov_lzz = []
    
    def IMU_callback(self,msg:Imu):
        self.ax = msg.angular_velocity.x
        self.ay = msg.angular_velocity.y
        self.az = msg.angular_velocity.z
        self.lx = msg.linear_acceleration.x
        self.ly = msg.linear_acceleration.y
        self.lz = msg.linear_acceleration.z
        self.data = 10
        
        self.i = self.i + 1
        if self.i == self.i % self.data:
            self.a_ax = (np.append(self.a_ax, self.ax)).tolist()
            self.a_ay = (np.append(self.a_ay, self.ay)).tolist()
            self.a_az = (np.append(self.a_az, self.az)).tolist()
            self.l_lx = (np.append(self.l_lx, self.lx)).tolist()
            self.l_ly = (np.append(self.l_ly, self.ly)).tolist()
            self.l_lz = (np.append(self.l_lz, self.lz)).tolist()
           
        if len(self.a_ax) == self.data-1:
            self.cov_axx = (np.cov(self.a_ax,self.a_ax)[0][1]).tolist()
            self.cov_axy = (np.cov(self.a_ax,self.a_ay)[0][1]).tolist()
            self.cov_axz = (np.cov(self.a_ax,self.a_az)[0][1]).tolist()
            self.cov_ayy = (np.cov(self.a_ay,self.a_ay)[0][1]).tolist()
            self.cov_ayz = (np.cov(self.a_ay,self.a_az)[0][1]).tolist()
            self.cov_azz = (np.cov(self.a_az,self.a_az)[0][1]).tolist()
            
            self.cov_lxx = (np.cov(self.l_lx,self.l_lx)[0][1]).tolist()
            self.cov_lxy = (np.cov(self.l_lx,self.l_ly)[0][1]).tolist()
            self.cov_lxz = (np.cov(self.l_lx,self.l_lz)[0][1]).tolist()
            self.cov_lyy = (np.cov(self.l_ly,self.l_ly)[0][1]).tolist()
            self.cov_lyz = (np.cov(self.l_ly,self.l_lz)[0][1]).tolist()
            self.cov_lzz = (np.cov(self.l_lz,self.l_lz)[0][1]).tolist()

            msg.angular_velocity_covariance = [self.cov_axx, self.cov_axy,self.cov_axz,self.cov_axy,self.cov_ayy,self.cov_ayz,self.cov_axz,self.cov_ayz,self.cov_azz]
            msg.linear_acceleration_covariance = [self.cov_lxx, self.cov_lxy,self.cov_lxz,self.cov_lxy,self.cov_lyy,self.cov_lyz,self.cov_lxz,self.cov_lyz,self.cov_lzz]
        self.ca = msg.angular_velocity_covariance
        self.cl = msg.linear_acceleration_covariance
        
    def timer_callback(self):
        pass
        self.msg.angular_velocity_covariance = (self.ca)
        # # print(type(self.msg.angular_velocity_covariance[0]))
        self.msg.linear_acceleration_covariance = (self.cl)
        self.msg.angular_velocity.x = self.ax
        self.msg.angular_velocity.y = self.ay
        self.msg.angular_velocity.z = self.az
        self.msg.linear_acceleration.x = self.lx
        self.msg.linear_acceleration.y = self.ly
        self.msg.linear_acceleration.z = self.lz
        # # print('----')
        self.publisher_.publish(self.msg)

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = Trajectory_publisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()