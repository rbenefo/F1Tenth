#!/usr/bin/env python2.7
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class reactive_follow_gap:
    def __init__(self):
        #Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback, queue_size = 1) #TODO
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size = 1)
        self.angles = None
        self.angles_initialized = False
        
    def preprocess_lidar(self, data, window_size = 7):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        #initialize data
        steering_viewport = 70 #in degrees
        ranges = np.array(data.ranges)
        if not self.angles_initialized:
            min_angle = data.angle_min
            max_angle = data.angle_max
            self.angles = np.linspace(min_angle, max_angle, ranges.shape[0])
            self.angles_initialized = True
            self.good_angle_idx = np.where(np.logical_and(self.angles > np.radians(-steering_viewport), self.angles < np.radians(steering_viewport)))
            self.angles = self.angles[self.good_angle_idx]

        #set Nan's to 0
        ranges[np.isnan(ranges)] = 0
        goodIdx = np.isfinite(ranges)
        #set InF's to 5 (don't set to 0 so that we don't remove legitimate gaps)
        ranges[~goodIdx] = 5

        kernel = np.ones((1,window_size))/window_size
        kernel = kernel[0,:]
        smoothed_ranges =np.convolve(ranges,kernel,'same')
        smoothed_ranges[~goodIdx] = 0
        smoothed_ranges = np.clip(smoothed_ranges, 0, 3)
        smoothed_ranges = smoothed_ranges[self.good_angle_idx]
        return smoothed_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        temp_arr = np.copy(free_space_ranges)
        temp_arr[np.nonzero(temp_arr)] = 2
        split = np.split(np.arange(free_space_ranges.shape[0]), np.where(np.abs(np.diff(temp_arr)) >= 1)[0]+1)
        
        sorted_split = sorted(split, key=len, reverse=True)
        for i in range(len(sorted_split)):
            if np.any(free_space_ranges[sorted_split[i]]):
                return np.min(sorted_split[i]), np.max(sorted_split[i]+1)
    

    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	Naive: Choose the furthest point within ranges and go there
        """

        return self.angles[np.argmax(ranges[start_i:end_i])+start_i]
        
    
    def set_bubble(self, ranges, closest_point_idx, rb = 0.6):
        """Rb is bubble radius"""
        angle = self.angles[closest_point_idx]
        dtheta = np.arctan2(rb, ranges[closest_point_idx])

        bubble_idx = np.where(np.logical_and(self.angles > angle-dtheta, self.angles < angle+dtheta))

        ranges[bubble_idx] = 0

        return ranges

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        proc_ranges = self.preprocess_lidar(data)

        #Find closest point to LiDAR
        closest_point_idx = np.argmin(proc_ranges[np.nonzero(proc_ranges)]) #nonzero
        # rospy.loginfo("closest_point_idx {}".format(closest_point_idx))


        #Eliminate all points inside 'bubble' (set them to zero) 

        bubbled_ranges = self.set_bubble(proc_ranges, closest_point_idx)

        #Find max length gap

        gap_start, gap_end = self.find_max_gap(bubbled_ranges)
        # rospy.loginfo("gap_start {}, gap_end {}".format(gap_start, gap_end))

        #Find the best point in the gap 
        desired_angle = self.find_best_point(gap_start, gap_end,bubbled_ranges)

        #Publish Drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = desired_angle
        drive_msg.drive.speed = 1.5 #slow constant velocity for now
        rospy.loginfo("desired_angle {}".format(desired_angle))

        self.drive_pub.publish(drive_msg)




def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)