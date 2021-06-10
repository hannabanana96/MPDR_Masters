#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
import spidev
import tf
import time
import math
from math import sin, cos, pi
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
#import geometry_msgs.msg
import RPi.GPIO as GPIO

# For resetting the encoders
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(22, GPIO.OUT)
GPIO.output(22,0)

WHEEL_RADIUS = 0.115 # Meters  
ROBOT_RADIUS = 0.515 # Meters  

# (Circumference_of_wheel) / (# of ticks per one rev)(nut multiplier)
# Wheel encoder is measuring a nut that spins on the shaft of the moter, 
#  so there is nut multiplier. The nut makes about 19 revolutions per
#  one wheel revolution. (POTENTIAL SOURCE OF ERROR)
DISTANCE_PER_TICK = (2 * 3.14159265 * WHEEL_RADIUS) / (16383 * 19); 


def linear_transform(DOMAIN,RANGE,debug=0):
    scale = (RANGE[1]-RANGE[0])/(DOMAIN[1]-DOMAIN[0])
    DOMAIN_SHIFT = DOMAIN[0]+(DOMAIN[1]-DOMAIN[0])/2
    RANGE_SHIFT = RANGE[0]+(RANGE[1]-RANGE[0])/2

    function = lambda x:scale*(x - DOMAIN_SHIFT) + RANGE_SHIFT
    if debug != 0:
        print("scale: " + str(scale))
        print("RANGE_SHIFT: " + str(RANGE_SHIFT))
        print("DOMAIN_SHIFT: " + str(DOMAIN_SHIFT))
    return function

class Encoder:
    def __init__(self):
        self.encoder_pub = rospy.Publisher('enc_odom',Odometry, queue_size=50)
        self.r = rospy.Rate(100)
        
        # Initial SPI
        self.lw_spi = spidev.SpiDev()
        self.lw_spi.open(0,0)   
        self.lw_spi.max_speed_hz = 5000
        self.lw_spi.mode = 0b01

        self.rw_spi = spidev.SpiDev()
        self.rw_spi.open(0,1) 
        self.rw_spi.max_speed_hz = 5000
        self.rw_spi.mode = 0b01

        # Convert 14bit values from encoders to degrees for testing
        self.data2degree = linear_transform((0,16383),(0,360))

        self.prev_tick_L = 0
        self.prev_tick_R = 0
    
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()


        self.list_of_bytes = [0xFF,0xFF]
        GPIO.output(22,1)

    # CHANGE THIS TO TICK VALUES - 14BITS/2 ETC
    # 14 bits = 16383 = 360 degrees
    # 16383/2 = 8920 (rounded up = 180 degrees
    # 0 = 0 degrees
    def angle2tick(self, end, start):
        diff = end - start
        if diff > 8920 and start < end:
            return diff - 16383
        elif diff < -8920 and start > end:
            return diff + 16383
        else:
            return diff

    
    # SPI Decoding - Error bit
    def get_error_bit(self, byte):
        error_bit = (~(0b10111111) & byte) >> 6
        # Check if there was an error
        if error_bit == 1:     
            return False  # Error was present
        else:
            return True   # Error not present

    # SPI Decoding - Calculate paridty bit for command message
    def get_pard_bit(self, byte):
        pard_bit = (~(0b10111111) & byte) >> 7

        SUM = 0
        for i in range(7):
            bit_check = (byte >> i) & 0x00000001
            if bit_check == 1:
                SUM += 1

        pard_bit = (~(0b01111111) & byte) >> 7
        if pard_bit == 0 and SUM % 2 == 0:
            return True
        elif pard_bit == 1 and SUM % 2 != 0:
            return True
        else:
            return False

    # Gets data from encoders 
    def get_data(self, list_of_2bytes):
        if not (self.get_pard_bit(list_of_2bytes[0]) or self.get_error_bit(list_of_2bytes[0])):
            print("bad get data bad dog")
            return 0xFFFF
        else:
            data = ~(0b11000000) & list_of_2bytes[0]
            data = (data << 8) + list_of_2bytes[1]
            return data

class RobotPosition():
    def __init__(self):

        # Initial values for odom frame
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.vx = 0.1
        self.vy = -0.1
        self.vth = 0.1

def main(encoder, robot):
    # Set up odometry broadcaster       
    odom_broadcaster = tf.TransformBroadcaster()

    i = 0 
    while not rospy.is_shutdown():
        current_time = rospy.Time.now()

        # Get motor position data from encoders
        encoder.rw_spi.xfer3(encoder.list_of_bytes,2) 
        raw_data_r = encoder.rw_spi.readbytes(2)
        curr_tick_r = encoder.get_data(raw_data_r)      # angle from encoder

        encoder.lw_spi.xfer3(encoder.list_of_bytes,2)
        raw_data_l = encoder.lw_spi.readbytes(2)
        curr_tick_l = encoder.get_data(raw_data_l)      #angle from encoder

        # Checking for errors, if error do this, we want to get rid of bad data
        if curr_tick_r == 0xFFFF:
            curr_tick_r = encoder.prev_tick_R
            print("bad R") 
        if curr_tick_l == 0xFFFF:
            curr_tick_l = encoder.prev_tick_L
            print("bad L")


        # Calculating the change in encoder tick values
        deltaLeftTicks = encoder.angle2tick(encoder.prev_tick_L, curr_tick_l)
        deltaRightTicks = -encoder.angle2tick(encoder.prev_tick_R, curr_tick_r)
        
        # Debug
        # Should be positive when moving forward       
        #print("deltaLeftTicks: " + str(deltaLeftTicks))
        #print("deltaRightTicks: " + str(deltaRightTicks))

        # Calculating the velocity of the wheels based on encoder ticks
        dt = (current_time - encoder.last_time).to_sec()
        vel_wheel_L = (deltaLeftTicks * DISTANCE_PER_TICK) / dt 
        vel_wheel_R = (deltaRightTicks * DISTANCE_PER_TICK) / dt

        # Calculating the robot's linear and angular velocities from wheel velocities
        robot.vx = ((vel_wheel_R + vel_wheel_L) / 2)
        robot.vy = 0;
        robot.vth = -((vel_wheel_R - vel_wheel_L)/ROBOT_RADIUS) #added negative
        
        # Compute odometry in a typical way given the velocities of the robot
        # Computing the robot's change in position and angle 
        delta_x = (robot.vx * cos(robot.th) - robot.vy * sin(robot.th)) * dt
        delta_y = (robot.vx * sin(robot.th) + robot.vy * cos(robot.th)) * dt
        delta_th = robot.vth * dt

        # In relation to the global coordinate frame
        robot.x += delta_x
        robot.y += delta_y
        robot.th += delta_th
        
        # Debug
        #print("robot.x: " + str(robot.x))
        #print("robot.y: " + str(robot.y))
        #print("robot.th, angular pos: " + str(robot.th))

        # Since all odometry is 6DOF we'll need a quaternion created from yaw
        # this is different the C++ code (they use createQuaternionMsgfromYaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, robot.th)

        # First, we'll publish the transform over tf
        odom_broadcaster.sendTransform(
            (robot.x, robot.y, 0.0),
            odom_quat,                      
            rospy.Time.now(),               # stamp
            "base_link",                    # child_frame_id
            "enc_odom_frame"                # frame_id
        )

        # Next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "enc_odom_frame" 

        # set the position
        odom.pose.pose.position.x = robot.x
        odom.pose.pose.position.y = robot.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(*odom_quat)
        #odom.pose.pose = Pose(Point(robot.x, robot.y, 0.0), Quaternion(*odom_quat))


        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = robot.vx
        odom.twist.twist.linear.y = robot.vy
        odom.twist.twist.angular.z = robot.vth
        #odom.twist.twist = Twist(Vector3(robot.vx, robot.vy, 0), Vector3(0, 0, robot.vth))

        # publish the message
        encoder.encoder_pub.publish(odom)
        
        encoder.prev_tick_L = curr_tick_l
        encoder.prev_tick_R = curr_tick_r
        encoder.last_time = current_time
        
        encoder.r.sleep()

if __name__ == "__main__":
  
  try:
    rospy.init_node('encoders')
    encoder = Encoder()
    robot = RobotPosition()
    main(encoder, robot)

  except:
    print("Failed to run encoder script")
    

