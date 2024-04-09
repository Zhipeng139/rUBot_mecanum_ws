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
    take_action(regions)


def take_action(regions):
    msg = Twist()
    linear_x = 0
    linear_y = 0
    angular_z = 0

    state_description = ''

    if regions['front'] > 2*d and regions['r_fright'] > 2*d and regions['right'] > 2*d and regions['r_bright'] > 2*d:
        state_description = 'case 1 - nothing'
        linear_x = vx * vf
        angular_z = 0

    elif regions['front'] < 1.5 *d:
        if regions['front'] < d:
            if regions["left"] < d:
                #linear_x = vx * vf 
                angular_z = wz * vf
                #linear_y = vx * vf
            else:
                state_description = 'case 2 - front'
                linear_x = 0
                #angular_z = wz * vf
                linear_y = vx * vf

        elif (regions['right'] < 1.5 * d) or (regions['r_fright'] < 1.5 * d):
            state_description = 'case 3 - front-right but far'
            linear_x = vx * vf 
            linear_y = vx * vf
        else:
            state_description = 'case 5 - front- but far'
            linear_x = vx * vf
            angular_z = 0
    elif (regions['right'] < 1.5*d) or (regions['r_fright'] < 1.5*d) or (regions['r_bright'] < 1.5*d):
        if (regions['right'] < d) or (regions['r_fright'] < d):
            state_description = 'case 6 - right '
            linear_x = 0
            linear_y = vx * vf 
        elif (regions['r_bright'] < d):
            linear_x = vx * vf 
            linear_y = vx * vf
        else:
            state_description = 'case 7 between'
            linear_x = vx * vf
            angular_z = 0
    else:
        state_description = 'case 8 ?'
        #linear_x = vx * vf * 0.5
        #angular_z = - wz * vf
        linear_y = - vx * vf 
    rospy.loginfo(state_description)
    msg.linear.x = linear_x
    msg.angular.z = angular_z
    msg.linear.y = linear_y
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

    rospy.init_node('wall_follower')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    rospy.on_shutdown(shutdown)
    rate = rospy.Rate(25)

    d= rospy.get_param("~distance_laser")
    vx= rospy.get_param("~forward_speed")
    wz= rospy.get_param("~rotation_speed")
    vf= rospy.get_param("~speed_factor")
    
    
if __name__ == '__main__':
    try:
        main()
        rospy.spin()
    except rospy.ROSInterruptException:
        shutdown()

    


if __name__ == '__main__':
    main()
