#!/usr/bin/env python2.7

import rospy
from sensor_msgs.msg import LaserScan


def callback(msg):
    timestamp = msg.header.stamp
    ranges = msg.ranges
    intensities = msg.intensities
    rospy.loginfo("timestamp {}".format(timestamp))

def main():
    rospy.init_node('scan_subscriber')
    print("inited")
    rospy.Subscriber("/scan", LaserScan, callback)
    rospy.spin()

if __name__ == "__main__":
    main()
