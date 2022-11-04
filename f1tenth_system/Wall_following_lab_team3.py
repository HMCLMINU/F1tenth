#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry


#TO DO: Tune parameters
#PID CONTROL PARAMS
kp = 0.8
kd = 0.06
ki = 0.00005
servo_offset = 0.0
prev_error = 0.0 
integral = 0.0


#WALL FOLLOW PARAMS 
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.50 # meters
DESIRED_DISTANCE_LEFT = 0.50
PI = 3.14159265358979
CAR_LENGTH = 0.50 # TfollowLeftraxxas Rally is 20 inches or 0.5 meters

class WallFollow:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        #Topics & Subs, Pubs
        self.stop_signal = 0
        self.velocity_sub = rospy.Subscriber("/vesc/odom", Odometry, self.callback_vel)#: Subscribe to VESC
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback)#: Subscribe to LIDAR
        self.drive_pub = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/output', AckermannDriveStamped, queue_size=10)#: Publish to drive
    
        self.prev_error = prev_error
        self.integral = integral
    
    def callback_vel(self,data):
        #TO DO: Subscribe Current Velocity
        self.real_velocity = data.twist.twist.linear.x

    def getRange(self, data):
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # Outputs length in meters to object with angle in lidar scan field of view
        #make sure to take care of nans etc.
        #TODO: implement
        laser_ranges = data.ranges
        laser_ranges = laser_ranges[600:710]
        return 0.0


    def pid_control(self, error, velocity):
    
        #TODO: Use kp, ki & kd to implement a PID controller
        #       Example:
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"

        time = 0.3

        P_control = kp * error
        I_control = self.integral + ki * error * time
        D_control = kd * (error - self.prev_error) / time 
        
        u = P_control + I_control + D_control

        #print(P_control)
        #print(I_control)
        #print(D_control)

        self.integral = I_control
        self.prev_error = error
        #print(u * 180 / PI)

        # if abs(u) > 20 * PI / 180 :
        #     velocity = 0.25 * velocity
        # elif abs(u) > 10 * PI / 180:
        #     velocity = 0.4  * velocity
        # else:
        #     velocity = 0.8  * velocity

        #print(velocity)
        drive_msg.drive.steering_angle = 0.0 # u
        # print("{}".format(velocity))
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)
        
        
      

    def lidar_callback(self, data):

        #TO DO:  
        #       1. Get LiDAR message
        #       2. Calculate length to object with angle in lidar scan field of view
        #          and make sure to take care of nans etc.
        #       3. Based on length to object, callback 'pid_control' 
        self.lidar = data.ranges
        NUM_RANGES = len(self.lidar)
        distance_b = self.lidar[(NUM_RANGES * 3) // 4]
        distance_a = self.lidar[(NUM_RANGES * 5) // 8]

        d1 = self.lidar[(NUM_RANGES * 1) // 4]
        d2 = self.lidar[(NUM_RANGES * 3) // 4]
        d3 = self.lidar[(NUM_RANGES * 1) // 2]



        alpha = np.arctan((distance_a * np.sqrt(2) / 2 - distance_b) / (distance_a * np.sqrt(2) / 2) )

        Dt = distance_b * np.cos(alpha)
        Dt1 = Dt + 0.7 * np.sin(alpha)

        Et = Dt1 - DESIRED_DISTANCE_LEFT

        dis_front = self.lidar[(NUM_RANGES * 1) // 2]

        # if dis_front < 0.3:
        #     velocity = 0.1
        # else:
        #     velocity = 1
        # TTC = 1
        
        v = max(self.real_velocity, 0.8)
        print("{}".format(d3 / (v+0.001+0.07)))
        TTC = d3 / (v+0.001+0.07)
        # if d1 < 0.6 and d2 < 0.6 and d3 < 0.6:
        #     TTC = d3 / (v+0.001+0.07)
            
        
        
        if TTC < 1 or self.stop_signal:
            self.stop_signal = True
            velocity = -1.5
        else:
            velocity = 1.5
        print("{}".format(velocity))
        self.pid_control(Et , velocity)



def main(args):

    rospy.init_node("WallFollow_node", anonymous=False)
    wf = WallFollow()
    rospy.sleep(1)
    rospy.spin()


if __name__=='__main__':
	main(sys.argv)
