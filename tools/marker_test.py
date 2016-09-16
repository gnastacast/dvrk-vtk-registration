#!/usr/bin/env python

import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math

topic = 'visualization_marker_array'
publisher = rospy.Publisher(topic, MarkerArray, queue_size=10)

rospy.init_node('register')

markerArray = MarkerArray()

marker_id = 0
for i in range(0, 9):
  for j in range(0, 6):
    if j % 2 != i % 2:
      continue
    marker = Marker()
    marker.header.frame_id = "/chessboard"
    marker.type = marker.CUBE
    marker.action = marker.ADD
    marker.scale.x = 0.0254
    marker.scale.y = 0.0254
    marker.scale.z = 0.0001
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.pose.orientation.w = 0.0
    marker.pose.orientation.x = 0.707
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.707
    marker.pose.position.y = 0.0254 * (j+.5) - 0.0254 
    marker.pose.position.z = 0.0254 * (i+.5) - 0.0254 
    marker.id = marker_id
    markerArray.markers.append(marker)
    marker_id +=1

while not rospy.is_shutdown():
   # Publish the MarkerArray
   publisher.publish(markerArray)
   rospy.sleep(0.01)