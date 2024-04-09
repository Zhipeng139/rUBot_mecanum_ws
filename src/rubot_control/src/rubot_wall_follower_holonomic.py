#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time

pub = None
d = 0
vx = 0
wz = 0
vf = 0
vy = 0
last_obs = 0  # 0 nothing, -1 left, 1 right

isScanRangesLengthCorrectionFactorCalculated = False
scanRangesLengthCorrectionFactor = 2


def clbk_laser(msg):
    # En la primera ejecucion, calculamos el factor de correcion
    global isScanRangesLengthCorrectionFactorCalculated
    global scanRangesLengthCorrectionFactor

    if not isScanRangesLengthCorrectionFactorCalculated:
        scanRangesLengthCorrectionFactor = len(msg.ranges) / 360
        isScanRangesLengthCorrectionFactorCalculated = True

    r_bright_min = int(30 * scanRangesLengthCorrectionFactor)
    r_bright_max = int(90 * scanRangesLengthCorrectionFactor)

    right_min = int(90 * scanRangesLengthCorrectionFactor)
    right_max = int(120 * scanRangesLengthCorrectionFactor)

    r_fright_min = int(120 * scanRangesLengthCorrectionFactor)
    r_fright_max = int(170 * scanRangesLengthCorrectionFactor)

    front_min = int(170 * scanRangesLengthCorrectionFactor)
    front_max = int(190 * scanRangesLengthCorrectionFactor)

    l_fright_min = int(190 * scanRangesLengthCorrectionFactor)
    l_fright_max = int(240 * scanRangesLengthCorrectionFactor)

    left_min = int(240 * scanRangesLengthCorrectionFactor)
    left_max = int(270 * scanRangesLengthCorrectionFactor)

    l_bright_min = int(270 * scanRangesLengthCorrectionFactor)
    l_bright_max = int(330 * scanRangesLengthCorrectionFactor)

    back_min = int(330 * scanRangesLengthCorrectionFactor)
    back_max = int(30 * scanRangesLengthCorrectionFactor)

    regions = {
        'r_bright':  min(min(msg.ranges[r_bright_min:r_bright_max]), 3),
        'right':  min(min(msg.ranges[right_min:right_max]), 3),
        'r_fright': min(min(msg.ranges[r_fright_min:r_fright_max]), 3),
        'front':  min(min(msg.ranges[front_min:front_max]), 3),
        'l_fright': min(min(msg.ranges[l_fright_min:l_fright_max]), 3),
        'left':  min(min(msg.ranges[left_min:left_max]), 3),
        'l_bright': min(min(msg.ranges[l_bright_min:l_bright_max]), 3),
        'back': min(min(msg.ranges[back_min:int(360*scanRangesLengthCorrectionFactor)]), min(msg.ranges[0: back_max]), 3),
    }

    take_action(regions, last_obs)


def take_action(regions, last_obs):
    msg = Twist()
    linear_x = 0
    linear_y = 0
    angular_z = 0

    state_description = ''

    if regions['front'] > d and regions['r_fright'] > 2*d and regions['right'] > 2*d and regions['r_bright'] > 2*d and regions['l_fright'] > 2*d and regions['left'] > 2*d and regions['l_bright'] > 2*d and regions['back'] > 2*d:
        state_description = 'case 1 - nothing'
        linear_x = vx
        angular_z = 0
        last_obs = 0
    elif regions['front'] < d:
        if last_obs == 0 or last_obs == 1:
            state_description = 'case 2 - obs in the front or front and right'
            linear_x = 0
            linear_y = vy
            last_obs = 1
        elif last_obs == -1:
            state_description = 'case 3 - front and obs in the left'
            linear_x = -vx
            linear_y = 0
            last_obs = -1
    elif regions['back'] < d:
        if last_obs == 0 or last_obs == -1:
            state_description = 'case 4 - obs in the back or back and left'
            linear_x = 0
            linear_y = -vy
            last_obs = -1
        elif last_obs == 1:
            state_description = 'case 5 - back and obs in the right'
            linear_x = vx
            linear_y = 0
            last_obs = 1
    elif regions['right'] < d:
        state_description = 'case 8 - right'
        linear_x = vx
        linear_y = 0
        angular_z = 0
        last_obs = 1
    elif regions['left'] < d:
        state_description = 'case 9 - left'
        linear_x = -vx
        linear_y = 0
        angular_z = 0
        last_obs = -1
    elif regions['l_fright'] < d or regions['r_fright'] < d:
        state_description = 'case 6 - l_fright'
        linear_x = 0
        linear_y = 0
        angular_z = wz * vf
        last_obs = 1
    elif regions['l_bright'] < d or regions['r_bright'] < d:
        state_description = 'case 7 - r_bright'
        linear_x = 0
        linear_y = 0
        angular_z = -wz * vf
        last_obs = 1
    else:
        state_description = 'case 10 - Far'
        if last_obs == 1:
            linear_x = 0
            linear_y = vy
            angular_z = 0
        elif last_obs == -1:
            linear_x = 0
            linear_y = -vy
            angular_z = 0
        elif last_obs == 0:
            linear_x = vx
            linear_y = 0
            angular_z = 0

    rospy.loginfo(state_description)
    rospy.loginfo(f"Current last_obs: { last_obs}")
    msg.linear.x = linear_x
    msg.linear.y = linear_y
    msg.angular.z = angular_z
    pub.publish(msg)
    rate.sleep()


def shutdown():
    msg = Twist()
    msg.linear.x = 0
    msg.linear.y = 0
    msg.angular.z = 0
    pub.publish(msg)
    rospy.loginfo("Stop rUBot")


def main():
    global pub
    global sub
    global rate
    global d
    global vx
    global wz
    global vf
    global vy
    global last_obs

    rospy.init_node('wall_follower')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    rospy.on_shutdown(shutdown)
    rate = rospy.Rate(25)
    last_obs = 0

    d = rospy.get_param("~distance_laser")
    vx = rospy.get_param("~forward_speed")
    wz = rospy.get_param("~rotation_speed")
    vf = rospy.get_param("~speed_factor")
    vy = rospy.get_param("~lateral_speed")

if __name__ == '__main__':

    try:
        main()
        rospy.spin()
    except rospy.ROSInterruptException:
        shutdown()


if __name__ == '__main__':
    main()
