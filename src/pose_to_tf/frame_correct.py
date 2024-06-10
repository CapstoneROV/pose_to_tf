#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2

def cloud_callback(cloud_msg):
    # Assuming this is the wrong one due to the topic name in recording the rosbag
    if cloud_msg.header.frame_id == 'map':
        cloud_msg.header.frame_id = 'ping360_link'  # Correcting the frame_id
        corrected_pub.publish(cloud_msg)  # Publish the corrected message

rospy.init_node('fix_frame_id')

# Subscriber to the specific PointCloud2 topic
wrong_cloud_sub = rospy.Subscriber('/filtered_msis', PointCloud2, cloud_callback)

# Publisher for the modified PointCloud2
corrected_pub = rospy.Publisher('/corrected_frame_cloud', PointCloud2, queue_size=10)

rospy.spin()