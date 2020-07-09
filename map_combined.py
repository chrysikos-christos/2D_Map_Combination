#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from nav_msgs.msg import OccupancyGrid

slam_map = OccupancyGrid()
projected_map = OccupancyGrid()

def callback_map(data):
    global slam_map 
    slam_map = data
    # rospy.loginfo(slam_map)
    mix()

def callback_projected_map(data):
    global projected_map
    projected_map = data
    # rospy.loginfo(projected_map)
    mix()

def mix():
    global slam_map, projected_map, pub
    
    dim1 = (slam_map.info.height,slam_map.info.width)
    slam_map_img = np.reshape(np.array(slam_map.data),dim1).astype('uint8')

    dim2 = (projected_map.info.height,projected_map.info.width)
    projected_map_img = np.reshape(np.array(projected_map.data),dim2).astype('uint8')
    projected_map_img = cv2.resize(projected_map_img,(slam_map.info.width,slam_map.info.height),interpolation=cv2.INTER_AREA)
    
    kernel = np.ones((5,5),np.uint8)
    projected_map_img = cv2.morphologyEx(projected_map_img, cv2.MORPH_CLOSE, kernel)
    projected_map_img = cv2.blur(projected_map_img,(3,3))
    
    map_combined_data = []
    for i in range(slam_map.info.height):
        for j in range(slam_map.info.width):
            if (slam_map_img[i,j] != 255 and projected_map_img[i,j] != 255):
                cell = (slam_map_img[i,j] + projected_map_img[i,j])/2
                map_combined_data.append(cell.astype(np.int8))
            else:
                map_combined_data.append(-1)

    # map_combined_img = slam_map_img
    # map_combined_img = (2*slam_map_img + projected_map_img)/3
    # map_combined_data = []
    # for i in range(map_combined_img.shape[0]):
    #     for j in range(map_combined_img.shape[1]):
    #             map_combined_data.append(map_combined_img[i,j])
    

    # for i in range(len(map_combined_data)):
    #         if (map_combined_data[i] == 0):
    #             map_combined_data[i] = -1
    
    # rospy.loginfo(len(map_combined_data))
    map_combined = OccupancyGrid()
    map_combined.info = slam_map.info
    map_combined.header.frame_id = slam_map.header.frame_id
    map_combined.header.stamp = rospy.Time.now()
    map_combined.info.resolution = slam_map.info.resolution
    map_combined.info.width = slam_map.info.width
    map_combined.info.height = slam_map.info.height
    map_combined.info.origin = slam_map.info.origin
    map_combined.data = map_combined_data
    rospy.loginfo("map_combined: Updated!!!")
    # rospy.loginfo(map_combined.data)
    talker(map_combined)


def talker(map_combined):
    pub = rospy.Publisher('map_combined', OccupancyGrid, queue_size=10)
    pub.publish(map_combined)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("map", OccupancyGrid, callback_map)
    rospy.Subscriber("projected_map", OccupancyGrid, callback_projected_map)

    rospy.spin()

if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInterruptException:
        pass


