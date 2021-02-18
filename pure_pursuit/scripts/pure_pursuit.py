#!/usr/bin/env python2.7
import rospy
from geometry_msgs.msg import PoseStamped, PointStamped, Pose
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry, Path

from tf.transformations import quaternion_matrix
import numpy as np
# TODO: import ROS msg types and libraries

class PurePursuit(object):
    """
    The class that handles pure pursuit.
    """
    def __init__(self):
        #sbscribe to /pf/pose/odom for particle filter pose
        
        self.waypoints = np.genfromtxt('/home/mlab/rcws/logs/wp-2021-02-01-07-19-40.csv',delimiter=',')
        self.waypoints = self.waypoints[:,0:2]
        self.waypoints = self.waypoints[0:int(self.waypoints.shape[0]*0.90),:]

        self.lookahead = 1 # const
        


        
        pf_odom_topic = '/odom'
        drive_topic = '/drive'
        waypoint_topic = '/waypoint'
        self.odom_sub = rospy.Subscriber(pf_odom_topic, Odometry, self.pose_callback, queue_size = 1)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size = 1)
        self.waypoint_pub = rospy.Publisher(waypoint_topic, PointStamped, queue_size = 1)

        self.last_idx = 0


    def get_curr_waypoint(self, location):
        stop = False
        if self.last_idx >= self.waypoints.shape[0]:
            stop = True
            return None, stop

        dist = (self.waypoints[:,0]-location[0])**2+(self.waypoints[:,1]-location[1])**2
        # rospy.loginfo("location: {}".format(location))
        for i in range(self.waypoints.shape[0]-1, self.last_idx,-1):
            if dist[i] < self.lookahead:
                self.last_idx = i

                break           
        waypoint_pre = self.waypoints[self.last_idx]
        waypoint_post = self.waypoints[self.last_idx+1]

        waypoint = (waypoint_pre+waypoint_post)/2 #average
        return waypoint, stop

    def set_speed(self, angle):
        abs_angle = np.abs(angle)
        if abs_angle >= 0.4:
            speed = 0.5
            self.lookahead = 0.5
        elif abs_angle < 0.4 or abs_angle >= 0.2:
            speed = 0.75
            self.lookahead = 0.75
        else:
            speed = 1
            self.lookahead =1
        return speed
    
    def pose_callback(self, pose_msg):
        location = [pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y]
        quaternion = [pose_msg.pose.pose.orientation.x, pose_msg.pose.pose.orientation.y, pose_msg.pose.pose.orientation.z, pose_msg.pose.pose.orientation.w]
        rot_matrix = quaternion_matrix(quaternion)[0:3, 0:3]
        #print(rot_matrix)
        target_waypoint, stop = self.get_curr_waypoint(location)

        if not stop:
            waypoint_msg = PointStamped()
            waypoint_msg.header.stamp = rospy.Time.now()
            waypoint_msg.header.frame_id = "map"
            waypoint_msg.point.x = target_waypoint[0]
            waypoint_msg.point.y = target_waypoint[1]
            waypoint_msg.point.z = 0
            self.waypoint_pub.publish(waypoint_msg)

            # TODO: find the current waypoint to track using methods mentioned in lecture

            # TODO: transform goal point to vehicle frame of reference
            # TODO: calculate curvature/steering angle
            goal_point_world = target_waypoint-location
            goal_point_world = np.append(goal_point_world, 0)
            goal_point_body = np.matmul(np.linalg.inv(rot_matrix), goal_point_world)



            angle = 2*goal_point_body[1]/self.lookahead**2
            # rospy.loginfo("Dgoal: {}, Desired angle: {}".format(goal_point_body[0:2], angle))


            # TODO: publish drive message, don't forget to limit the steering angle between -0.4189 and 0.4189 radians
            angle = np.clip(angle, -0.4189, 0.4189)
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = rospy.Time.now()
            drive_msg.header.frame_id = "laser"
            drive_msg.drive.steering_angle = angle
            drive_msg.drive.speed = self.set_speed(angle)
        else:
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = rospy.Time.now()
            drive_msg.header.frame_id = "laser"
            drive_msg.drive.steering_angle = 0
            drive_msg.drive.speed = 0
        self.drive_pub.publish(drive_msg)




def main():
    rospy.init_node('pure_pursuit_node')
    pp = PurePursuit()
    rospy.spin()
if __name__ == '__main__':
    main()