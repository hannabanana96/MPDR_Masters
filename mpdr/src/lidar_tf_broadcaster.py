#!/usr/bin/env python3

import rospy
import tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped

def lidar_callback(msg):
    lidar_br = tf.TransformBroadcaster()
   
    current_time = rospy.Time.now()
    """
    lidar_quat = tf.transformations.quaternion_from_euler(0, 0, 0)
    lidar_trans = TransformStamped()
    lidar_trans.header.stamp = current_time
    lidar_trans.parent = "base_link"
    lidar_trans.child = "lidar"
    lidar_trans.time = current_time

    lidar_trans.transform.translation.x = 0.0025
    lidar_trans.transform.translation.y = 0
    lidar_trans.transform.translation.z = 0.1905
    lidar_trans.transform.rotation = lidar_quat

    lidar_br.sendTransform(lidar_trans)
    """

    lidar_br.sendTransform(
        #(-0.3575, 0, 0.1905),		# Lidars xyz displacement from base_link
        (-0.1397, 0, 0.1905),		# Lidars xyz displacement from base_link
        tf.transformations.quaternion_from_euler(0, 0, 3.14),
        rospy.Time.now(),
        "laser",
        "base_link"
    )


if __name__ == '__main__':
    rospy.init_node('lidar_tf_broadcaster')
    rospy.Subscriber('/scan', LaserScan, lidar_callback)
    rospy.spin()


