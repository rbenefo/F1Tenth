#!/usr/bin/env python2.7
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

#PID CONTROL PARAMS
kp = -15
kd = 5
ki = 0
servo_offset = 0.0
prev_error = 0.0 
error = 0.0
integral = 0.0

#WALL FOLLOW PARAMS
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.9 # meters
DESIRED_DISTANCE_LEFT = 0.55
VELOCITY = 2.00 # meters per second
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters

class WallFollow:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback, queue_size = 1)
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size = 1)

    def getRange(self, data, angle):
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # Outputs length in meters to object with angle in lidar scan field of view
        #make sure to take care of nans etc.
        #TODO: implement
        ranges = np.array(data.ranges)
        # rospy.loginfo("ranges {}".format(ranges[400:600]))
        angle_incr = data.angle_increment
        desired_idx = int((np.radians(angle)-data.angle_min)/angle_incr)
        # rospy.loginfo("angle {}, desired_idx {}, range {}".format(angle, desired_idx, ranges[desired_idx]))
        if np.isfinite(ranges[desired_idx]):
            return ranges[desired_idx]
        else:
            return None



    def pid_control(self, error):
        global integral
        global prev_error
        global kp
        global ki
        global kd
        deriv_error = error-prev_error
        prev_error = error
        
        integral += error
        #could implement integral dumping

        angle = np.radians(kp*error+kd*deriv_error+ki*integral)
        velocity = self.calc_speed(angle)
        rospy.loginfo("Error {}, Angle {}".format(error, np.degrees(angle)))


        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

    def calc_speed(self, angle):
        angle = np.abs(np.degrees(angle))
        if angle >= 0 and angle < 10:
            speed = 1.5
        elif angle >= 10 and angle < 20:
            speed = 1.0
        else:
            speed = 0.5
        return speed

    def followLeft(self, data, leftDist):
        #Follow left wall as per the algorithm 
        #TODO:implement

        zero_angle = 90
        b = self.getRange(data, zero_angle)
        
        # theta = np.random.randint(181,225)
        theta = 40
        a = self.getRange(data, zero_angle - theta)
        theta = np.radians(theta)
        if b is not None and a is not None:
            alpha = np.arctan2(a*np.cos(theta)-b, a*np.sin(theta))
            Dleft = b*np.cos(alpha)


            D_left_lookahead = Dleft+CAR_LENGTH*np.sin(alpha)

            # rospy.loginfo("alpha {}, DRight {}, b {}, a {}".format(alpha, Dleft, b, a))

            error = DESIRED_DISTANCE_LEFT - D_left_lookahead
            # rospy.loginfo("Error {}".format(error))

            return error
        else:
            return None

    def lidar_callback(self, data):
        """ 
        """

        error = self.followLeft(data, DESIRED_DISTANCE_LEFT)
        if error is not None:
            #send error to pid_control
            self.pid_control(error)

def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)