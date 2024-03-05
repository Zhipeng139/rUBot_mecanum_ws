#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan

def get_value (angle, factor):
    return int(angle * factor)

def callback(msg):
    # n = angle
    # n * (num_laser / 360) = angle_value
    num_laser = len(msg.ranges)
    laser_factor = num_laser / 360
    print ("Number of scan points: "+ str(num_laser))

    # values at 0 degrees
    print ("Distance at 0deg: " + str(msg.ranges[get_value(0, laser_factor)]))
    # values at 90 degrees
    print ("Distance at 90deg: " + str(msg.ranges[get_value(90, laser_factor)]))
    # values at 180 degrees
    print ("Distance at 180deg: " + str(msg.ranges[get_value(180, laser_factor)]))
    # values at 270 degrees
    print ("Distance at 270deg: " + str(msg.ranges[get_value(270, laser_factor)]))
    # values at 360 degrees
    print ("Distance at 360deg: " + str(msg.ranges[get_value(360, laser_factor)]))


rospy.init_node('scan_values')
sub = rospy.Subscriber('/scan', LaserScan, callback)
rospy.spin()