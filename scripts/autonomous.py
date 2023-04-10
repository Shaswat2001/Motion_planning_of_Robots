#!/usr/bin/env python3

import rospy
import math
import numpy as np
from motion_planning.msg import motion_planning
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose,Point,Quaternion,Twist
from tf.transformations import euler_from_quaternion

algorithm_path = None
nearest_vertex = None

def results_callback(data):

    global algorithm_path
    global nearest_vertex

    algorithm_path = data.path
    nearest_vertex = data.closest_vertex

class MoveForward:

    def __init__(self,pos_error):

        rospy.on_shutdown(self.shutdown)

        self.vel_pub = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
        self.pose_sub = rospy.Subscriber("/odom",Odometry,self.pose_callback)

        self.pos_error = pos_error
        self.current_pose = Odometry()
        self.rate = rospy.Rate(10)

    def pose_callback(self,pose_data):
        
        self.current_pose = pose_data
        self.current_pose.pose.pose.position.x = round(self.current_pose.pose.pose.position.x,4)
        self.current_pose.pose.pose.position.y = round(self.current_pose.pose.pose.position.y,4)

    def move(self,goal):

        goal_pose = Pose()
        goal_pose.position.x = goal[0]
        goal_pose.position.y = goal[1]

        vel_msg = Twist()

        while self.euclidean_distance(goal_pose) >= self.pos_error:

            vel_msg.linear.x = 0.2
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0

            # Publishing our vel_msg
            self.vel_pub.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.vel_pub.publish(vel_msg)

    def euclidean_distance(self,goal):

        pose = self.current_pose.pose.pose.position
        rospy.loginfo(f"The position x : {pose.x} and y : {pose.y}")
        return math.sqrt((pose.x-goal.position.x)**2 + (pose.y-goal.position.y)**2)

    def shutdown(self):

        rospy.loginfo("Stop")
        rospy.sleep(1)

class RotateClockWise:

    def __init__(self,theta_res):

        rospy.on_shutdown(self.shutdown)

        self.vel_pub = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
        self.pose_sub = rospy.Subscriber("/odom",Odometry,self.pose_callback)

        self.theta_res = theta_res
        self.current_pose = Odometry()
        self.rate = rospy.Rate(10)

    def pose_callback(self,pose_data):
        
        
        self.current_pose = pose_data
        self.current_pose.pose.pose.position.x = round(self.current_pose.pose.pose.position.x,4)
        self.current_pose.pose.pose.position.y = round(self.current_pose.pose.pose.position.y,4)

    def rotate(self,goal):

        goal_pose = Pose()
        goal_pose.position.x = goal[0]
        goal_pose.position.y = goal[1]

        vel_msg = Twist()

        while abs(self.angle_difference(goal_pose)) >= self.theta_res:

            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = -0.2

            # Publishing our vel_msg
            self.vel_pub.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.vel_pub.publish(vel_msg)

    def angle_difference(self,goal):

        pose = self.current_pose.pose.pose.position
        current_orientation = self.current_pose.pose.pose.orientation

        rospy.loginfo(f"The position x : {pose.x} and y : {pose.y}")
        goal_angle = np.rad2deg(math.atan2(goal.position.y - pose.y, goal.position.x - pose.x))%360
        quaternion = [current_orientation.x,current_orientation.y,current_orientation.z,current_orientation.w]
        
        current_angle = np.rad2deg(euler_from_quaternion(quaternion)[-1])%360

        rospy.loginfo(f"The current angle is {current_angle} and goal is {goal_angle}")
        return goal_angle - current_angle

    def shutdown(self):

        rospy.loginfo("Stop")
        rospy.sleep(1)

class RotateCounterClockWise:

    def __init__(self,theta_res):

        rospy.on_shutdown(self.shutdown)

        self.vel_pub = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
        self.pose_sub = rospy.Subscriber("/odom",Odometry,self.pose_callback)

        self.theta_res = theta_res
        self.current_pose = Odometry()
        self.rate = rospy.Rate(10)

    def pose_callback(self,pose_data):
        
        
        self.current_pose = pose_data
        self.current_pose.pose.pose.position.x = round(self.current_pose.pose.pose.position.x,4)
        self.current_pose.pose.pose.position.y = round(self.current_pose.pose.pose.position.y,4)

    def rotate(self,goal):

        goal_pose = Pose()
        goal_pose.position.x = goal[0]
        goal_pose.position.y = goal[1]

        vel_msg = Twist()

        while abs(self.angle_difference(goal_pose)) >= self.theta_res:

            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0.2

            # Publishing our vel_msg
            self.vel_pub.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.vel_pub.publish(vel_msg)

    def angle_difference(self,goal):

        pose = self.current_pose.pose.pose.position
        current_orientation = self.current_pose.pose.pose.orientation

        rospy.loginfo(f"The position x : {pose.x} and y : {pose.y}")
        goal_angle = np.rad2deg(math.atan2(goal.position.y - pose.y, goal.position.x - pose.x))%360
        quaternion = [current_orientation.x,current_orientation.y,current_orientation.z,current_orientation.w]
        
        current_angle = np.rad2deg(euler_from_quaternion(quaternion)[-1])%360

        rospy.loginfo(f"The current angle is {current_angle} and goal is {goal_angle}")
        return goal_angle - current_angle

    def shutdown(self):

        rospy.loginfo("Stop")
        rospy.sleep(1)

if __name__=="__main__":

    rospy.init_node("autonomous_node")

    while True:
        mp_sub = rospy.Subscriber("/motion_planning_results",motion_planning,results_callback)
        
        if algorithm_path is not None:
            mp_sub = None
            break

    



    
    
    
    
