import numpy as np


dist = np.load("dist.npy")
dist_sorted_idx = np.load("dist_sorted_idx.npy")

l = 0.5

waypoints = np.genfromtxt('/home/mlab/rcws/logs/wp-2021-02-01-07-19-40.csv',delimiter=',')
waypoints = waypoints[:,0:2]

waypoints = waypoints[0:int(waypoints.shape[0]*0.75),:]
print(waypoints.shape)


for i in range(last_idx, waypoints):
    
# dist_sorted = np.sort(dist)
# split_idx = dist_sorted.searchsorted(l)
# # rospy.loginfo("split idx {}".format(split_idx))
# dist_split = np.array_split(dist_sorted_idx, [split_idx])
# # rospy.loginfo("dist split shape {}".format(len(dist_split)))
# idx_within_circle = dist_split[0][-1]
# idx_outside_circle = dist_split[1][0]

# waypoint_pre = waypoints[idx_within_circle]
# waypoint_post = waypoints[idx_outside_circle]

# print(idx_within_circle)
# print(idx_outside_circle)


# waypoint = (waypoint_pre+waypoint_post)/2 #average, transform to vehicle reference
