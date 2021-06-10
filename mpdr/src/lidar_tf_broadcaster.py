#!/usr/bin/env python3

import rospy
import tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped

def lidar_callback(msg):
    lidar_br = tf.TransformBroadcaster()
   
    current_time = rospy.Time.now()

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


