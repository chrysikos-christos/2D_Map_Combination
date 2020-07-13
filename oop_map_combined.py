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
      
    def callback_projected_map(self, msg):
      self.projected_map = msg
      self.mix()

    def mix(self):
      # Convert maps to images
      dim1 = (self.slam_map.info.height,self.slam_map.info.width)
      slam_map_img = np.reshape(np.array(self.slam_map.data),dim1).astype('uint8')

      dim2 = (self.projected_map.info.height,self.projected_map.info.width)
      projected_map_img = np.reshape(np.array(self.projected_map.data),dim2).astype('uint8')
      
      # Crop Projected Map
      x0 = int(round(abs(self.projected_map.info.origin.position.x - self.slam_map.info.origin.position.x)/0.03))
      y0 = int(round(abs(self.projected_map.info.origin.position.y - self.slam_map.info.origin.position.y)/0.03))
      h = int(round(self.slam_map.info.height*0.05/0.03))
      w = int(round(self.slam_map.info.width*0.05/0.03))
      rospy.loginfo(slam_map_img.shape)
      cropped = projected_map_img[y0:y0+h, x0:x0+w]

      projected_map_img = cv2.resize(cropped,(self.slam_map.info.width,self.slam_map.info.height),interpolation=cv2.INTER_AREA)
      rospy.loginfo(projected_map_img.shape)

      # Maps Fusion
      map_combined_img = slam_map_img + 0.7*projected_map_img

      # cv2.imwrite("cropped_img.png", projected_map_img)
      # cv2.imwrite("slam_map_img.png", slam_map_img)
      # cv2.imwrite("map_combined_img.png", map_combined_img)
      
      map_combined_data = []
      for i in range(self.slam_map.info.height):
          for j in range(self.slam_map.info.width):
              if (map_combined_img[i,j] > 200):
                  map_combined_data.append(-1)
              elif (map_combined_img[i,j] > 100):
                  map_combined_data.append(100)
              else:
                  map_combined_data.append(map_combined_img[i,j].astype(np.int8))

      # Publishing the combined map
      self.map_combined.header.stamp = rospy.Time.now()
      self.map_combined.data = map_combined_data
      rospy.loginfo("map_combined: Updated!!!")
      # rospy.loginfo(map_combined.data)
      self.pub.publish(self.map_combined)

if __name__ == '__main__':
    rospy.init_node('map_combined')
    Map_combined()
    rospy.spin()