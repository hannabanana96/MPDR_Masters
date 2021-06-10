#!/usr/bin/env python3

import rospy
import tf
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped

def imu_callback(msg):
    imu_br = tf.TransformBroadcaster()
   
    current_time = rospy.Time.now()

    imu_br.sendTransform(
        #(-0.3575, 0, 0.1905),		# Lidars xyz displacement from base_link
        (-0.3048, -0.1397, 0),		# Lidars xyz displacement from base_link
        tf.transformations.quaternion_from_euler(0, 0, 0),
        rospy.Time.now(),
        "imu_frame",
        "base_link"
    )


if __name__ == '__main__':
    rospy.init_node('imu_tf_broadcaster')
    rospy.Subscriber('/bno08x/raw', Imu, imu_callback)
    rospy.spin()


