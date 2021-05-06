#!/usr/bin/env python  

import rospy
import tf

def handle_turtle_pose(msg, turtlename):
    br = tf.TransformBroadcaster()
    br.sendTransform(tf.StampedTransform(
						(-0.25, 0, 0),		# Lidars xyz displacement from base_link
                     	tf.transformations.quaternion_from_euler(0, 0, 0),  #rwy displacement
                     	rospy.Time.now(),
                     	"base_link",
                     	"laser")

if __name__ == '__main__':
    rospy.init_node('lidar_tf_broadcaster')
#    rospy.Subscriber('/pose' % turtlename,
#                     turtlesim.msg.Pose,
#                     handle_turtle_pose,
#                     turtlename)
#    rospy.spin()


