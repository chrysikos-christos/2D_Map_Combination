#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from random import choice

def talker():
    pub1 = rospy.Publisher('map', OccupancyGrid, queue_size=10)
    pub2 = rospy.Publisher('projected_map', OccupancyGrid, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    sequence = [-1,-1,-1,-1,-1,-1,-1,100,-1,-1,-1,-1,-1,-1,-1,100,100,-1,-1,-1,-1,-1,-1,-1]
    slam_map = OccupancyGrid()
    slam_map.info.width = 50
    slam_map.info.height = 70 
    slam_map.info.resolution = 1.0 # [m/cell]
    projected_map = OccupancyGrid()
    projected_map.info.width = 80
    projected_map.info.height = 100 
    projected_map.info.resolution = 1.0 # [m/cell]

    for i in range(slam_map.info.width * slam_map.info.height):
	    slam_map.data.append(choice(sequence))
    for i in range(projected_map.info.width * projected_map.info.height):
	    projected_map.data.append(choice(sequence))
    

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        slam_map.header.stamp = current_time
        slam_map.header.frame_id = 'map'
        projected_map.header.stamp = current_time
        projected_map.header.frame_id = 'map'
        # rospy.loginfo(type(slam_map.header.stamp.nsecs))
        # rospy.loginfo(len(projected_map.data))
        pub1.publish(slam_map)
        pub2.publish(projected_map)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass