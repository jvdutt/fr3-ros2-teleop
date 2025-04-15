#!/usr/bin/env python3

import hebi
from time import sleep, time
from matplotlib import pyplot as plt
import numpy as np


from scipy.spatial.transform import Rotation 
# import transforms3d as t3d

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion,TransformStamped
from std_msgs.msg import Header,String


from tf2_ros import TransformBroadcaster

CMD_RATE = 20 #hz
POSITION_SCALE = (0.01/CMD_RATE) *5 #max 1cm/s
ORIENTATION_SCALE = (np.pi/180)/CMD_RATE *10#max 1deg/s


class Teleop(Node):

    def __init__(self,group,fbk):
        super().__init__('teleop')

        #fbk init
        self.group = group
        self.fbk = fbk
        self.get_feeback()
        while self.fbk is None:
            self.get_feeback()

        #readio init
        self.buttons = [0]*8
        self.sliders = [0]*8

        #end effector init
        self.R_ee,self.t_ee = None,None

        #broadcater init
        self.tf_broadcaster = TransformBroadcaster(self)


        # self.publisher = self.create_publisher(PoseStamped, '/test', 10)
        self.publisher = self.create_publisher(PoseStamped, '/equilibrium_pose', 10)
        self.subscription = self.create_subscription(PoseStamped,'/franka_robot_state_broadcaster/current_pose',self.subcription_callback,10)

        sleep(1.0)
        self.timer = self.create_timer(1/CMD_RATE, self.timer_callback)
        print("started")

    def subcription_callback(self,msg):
        if self.R_ee is None:
            self.R_ee,self.t_ee = self.pose2mat(msg)
        self.tf_broadcaster.sendTransform(self.mat2tfpose(self.R_ee,self.t_ee))


    def timer_callback(self):
        self.read_io()

        t_change = np.array([-self.sliders[1],self.sliders[0],self.sliders[5]])*POSITION_SCALE
        self.t_ee = self.t_ee + t_change

        angles = [self.sliders[2]*ORIENTATION_SCALE,self.sliders[3]*ORIENTATION_SCALE,self.sliders[4]*ORIENTATION_SCALE]
        self.R_ee = self.R_ee@self.get_Rx(angles[0]) @ self.get_Ry(angles[1]) @ self.get_Rz(angles[2])

        pose_msg = self.mat2pose(self.R_ee,self.t_ee)
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'base'
        self.publisher.publish(pose_msg)
        


    def get_feeback(self):
        prev_fbk = self.fbk
        fbk = self.group.get_next_feedback(reuse_fbk=self.fbk)
        if fbk is None:
            self.fbk = prev_fbk
        else:
            self.fbk = fbk
        return self.fbk
    
    def read_io(self):
        self.get_feeback()
        for i in range(8):
            self.buttons[i] = self.fbk.io.b.get_int(i + 1).item()
        for i in range(8):
            self.sliders[i] = self.fbk.io.a.get_float(i + 1).item()

    def pose2mat(self,pose_msg):
        pos = np.array([pose_msg.pose.position.x,pose_msg.pose.position.y,pose_msg.pose.position.z])
        orient = np.array([pose_msg.pose.orientation.x,pose_msg.pose.orientation.y,pose_msg.pose.orientation.z,pose_msg.pose.orientation.w])
        R = Rotation.from_quat(orient).as_matrix()
        return R,pos
    
    def mat2pose(self,Rot,t):
        pose_msg = PoseStamped()
        pose_msg.pose.position.x = t[0]
        pose_msg.pose.position.y = t[1]
        pose_msg.pose.position.z = t[2]
        quat = Rotation.from_matrix(Rot).as_quat()
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]
        return pose_msg
    
    def mat2tfpose(self,Rot,t_):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base'
        t.child_frame_id = 'teleop'
        t.transform.translation.x = t_[0]
        t.transform.translation.y = t_[1]
        t.transform.translation.z = t_[2]
        quat = Rotation.from_matrix(Rot).as_quat()
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        return t

    
    def get_Rz(self,theta):
        return np.array([[np.cos(theta), -np.sin(theta), 0],
                         [np.sin(theta), np.cos(theta), 0],
                         [0, 0, 1]])
    def get_Ry(self,theta):
        return np.array([[np.cos(theta), 0, np.sin(theta)],
                         [0, 1, 0],
                         [-np.sin(theta), 0, np.cos(theta)]])
    def get_Rx(self,theta):
        return np.array([[1, 0, 0],
                         [0, np.cos(theta), -np.sin(theta)],
                         [0, np.sin(theta), np.cos(theta)]])

if __name__ == '__main__':

    lookup = hebi.Lookup()

    # Wait 2 seconds for the module list to populate
    sleep(2.0)

    family_name = "HEBI"
    module_name = "mobileIO"

    group = lookup.get_group_from_names([family_name], [module_name])

    if group is None:
        print('Group not found: Did you forget to set the module family and name above?')
        exit(1)

    fbk = hebi.GroupFeedback(group.size)

    rclpy.init()
    teleop = Teleop(group,fbk)
    rclpy.spin(teleop)
    teleop.destroy_node()
    rclpy.shutdown()


