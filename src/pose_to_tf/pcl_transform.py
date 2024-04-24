#!/usr/bin/env python
import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

def cloud_callback(cloud_msg, tf_buffer, pub):
    try:
        # Look up the transformation from ping360_link to map
        transform_stamped = tf_buffer.lookup_transform('map', 'ping360_link', rospy.Time(0))

        # Perform the transformation using tf2
        cloud_transformed = do_transform_cloud(cloud_msg, transform_stamped)

        # Publish the transformed point cloud
        pub.publish(cloud_transformed)
    except tf2_ros.TransformException as ex:
        rospy.logerr(str(ex))

def main():
    rospy.init_node('pointcloud_transformer', anonymous=True)

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    pub = rospy.Publisher('/filtered_msis', PointCloud2, queue_size=1)
    sub = rospy.Subscriber('/corrected_frame_cloud', PointCloud2, lambda msg: cloud_callback(msg, tf_buffer, pub))

    rospy.spin()

if __name__ == '__main__':
    main()