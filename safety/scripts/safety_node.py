#!/usr/bin/env python2.7

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
# TODO: import ROS msg types and libraries

class Safety(object):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        """
        One publisher should publish to the /brake topic with a AckermannDriveStamped brake message.
        One publisher should publish to the /brake_bool topic with a Bool message.
        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /odom topic to get the current speed of the vehicle.
        The subscribers should use the provided odom_callback and scan_callback as callback methods
        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0
        self.TTC_thresh = 0.3 #1 second TTC threshold

        self.init_publishers()
        self.init_subscribers()

    def init_publishers(self):
        self._brake_publisher = rospy.Publisher("/brake", AckermannDriveStamped, queue_size = 1)
        self._brake_bool_publisher = rospy.Publisher("/brake_bool", Bool, queue_size = 1)

    def init_subscribers(self):
        rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size = 1)
        rospy.Subscriber("/odom", Odometry, self.odom_callback, queue_size = 1)


    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # TODO: calculate TTC
        #first, grab ranges and generate angle vector
        if self.speed != 0:
            # delay = (rospy.Time.now()- scan_msg.header.stamp).to_sec()
            ranges = np.array(scan_msg.ranges)
            # ranges = ranges[np.isfinite(ranges)]
            min_angle = scan_msg.angle_min
            max_angle = scan_msg.angle_max
            increment = scan_msg.angle_increment
            angle_vec = np.arange(min_angle, max_angle, increment)
            dr = np.cos(angle_vec)*self.speed
            denom = np.where(dr < 0, 0.01, dr)
            TTC = ranges/denom
            rospy.loginfo("Rangemin {}, TTC {}".format(np.min(ranges), np.min(TTC)))

            #Publish brake bool and brake message in emergency
            if np.any(TTC < self.TTC_thresh):
                ack_msg = AckermannDriveStamped()
                ack_msg.drive.speed = 0.0
                brake_bool = Bool()
                brake_bool.data = True
                self._brake_publisher.publish(ack_msg)
                self._brake_bool_publisher.publish(brake_bool)
        else:
            brake_bool = Bool()
            brake_bool.data = False
            self._brake_bool_publisher.publish(brake_bool)





def main():
    rospy.init_node('safety_node')
    sn = Safety()
    rospy.spin()
if __name__ == '__main__':
    main()
