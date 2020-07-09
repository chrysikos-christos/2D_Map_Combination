#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from nav_msgs.msg import OccupancyGrid

class Map_combined:
    def __init__(self):
      self.slam_map = OccupancyGrid()
      self.projected_map = OccupancyGrid()
      self.map_combined = OccupancyGrid()
      self.pub = rospy.Publisher('map_combined', OccupancyGrid, queue_size=10)
      self.slam_map_sub = rospy.Subscriber("map", OccupancyGrid, self.callback_slam_map)
      self.projected_map_sub = rospy.Subscriber("projected_map", OccupancyGrid, self.callback_projected_map)
    

    def callback_slam_map(self, msg):
      self.slam_map = msg
      self.map_combined = self.slam_map
      self.pub.publish(self.map_combined)
      rospy.loginfo("map_combined: Updated!!!")
      

    def callback_projected_map(self, msg):
      self.projected_map = msg
      # rospy.loginfo(self.projected_map.header)
      self.mix()

    def mix(self):
      # Convert map to images
      dim1 = (self.slam_map.info.height,self.slam_map.info.width)
      slam_map_img = np.reshape(np.array(self.slam_map.data),dim1).astype('uint8')

      dim2 = (self.projected_map.info.height,self.projected_map.info.width)
      projected_map_img = np.reshape(np.array(self.projected_map.data),dim2).astype('uint8')
      
      # Image processing
      projected_map_img = cv2.resize(projected_map_img,(self.slam_map.info.width,self.slam_map.info.height),interpolation=cv2.INTER_AREA)
      
      kernel = np.ones((5,5),np.uint8)
      projected_map_img = cv2.morphologyEx(projected_map_img, cv2.MORPH_CLOSE, kernel)
      projected_map_img = cv2.blur(projected_map_img,(3,3))
      
      map_combined_data = []
      for i in range(self.slam_map.info.height):
          for j in range(self.slam_map.info.width):
              if (slam_map_img[i,j] != 255 and projected_map_img[i,j] != 255):
                  cell = (slam_map_img[i,j] + projected_map_img[i,j])/2
                  map_combined_data.append(cell.astype(np.int8))
              else:
                  map_combined_data.append(-1)

      # Publishing the combined map
      self.map_combined.header.stamp = rospy.Time.now()
      self.map_combined.data = map_combined_data
      rospy.loginfo("map_combined: Updated!!! mix")
      # rospy.loginfo(map_combined.data)
      self.pub.publish(self.map_combined)

if __name__ == '__main__':
    rospy.init_node('map_combined')
    Map_combined()
    rospy.spin()