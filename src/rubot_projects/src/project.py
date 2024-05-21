#!/usr/bin/env python3
import rospy
import rospkg
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import degrees, radians
import yaml
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import cv2 as cv
from rubot_project1_picture import TakePhoto
# from TrafficSignalsDetection_p2 import signal_detected
from keras.models import load_model  # TensorFlow is required for Keras to work
import cv2
import numpy as np


# def signal_detected(photo):
#     image = cv2.imread(photo)
#     cv2.imshow('Signal',image)
#     signal = "left"
#     return signal



# CAMERA can be 0 or 1 based on 

def find_arrow_direction(img):
    # Convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Apply edge detection
    gaussian = cv2.GaussianBlur(gray, (3, 3), 1)
    edges = cv2.Canny(gaussian, 150, 180)
    #plt.imshow(edges, cmap='gray')
    #plt.show()

    # Find contours
    contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Assume the largest contour is the arrow
    arrow_contour = max(contours, key=cv2.contourArea)

    # Find the bounding rectangle for the largest contour
    x, y, w, h = cv2.boundingRect(arrow_contour)
    
    # Draw the bounding rectangle
    #cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
    #plt.imshow(image)
    #plt.show()

    # Calculate the center of the arrow
    center = (x + w // 2, y + h // 2)

    # Determine direction based on the geometry of the arrow
    # Find the center of mass of the arrow
    M = cv2.moments(arrow_contour)
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])

    # Determine if the arrow is primarily horizontal or vertical
    if w > h:
        # Horizontal arrow
        direction = 'left' if cx < center[0] else 'right'
    else:
        # Vertical arrow
        direction = 'up' if cy < center[1] else 'down'

    return direction

def signal_detected(photo):
    img = cv2.imread(photo)
    image = cv2.resize(img, (224, 224), interpolation=cv2.INTER_AREA)
    image = np.asarray(image, dtype=np.float32).reshape(1, 224, 224, 3)
    # Normalize the image array
    image = (image / 127.5) - 1

    # Predicts the model
    prediction = model.predict(image)
    index = np.argmax(prediction)
    class_name = class_names[index]
    confidence_score = prediction[0][index]

    direction =  class_name[2:]
    print("direction: " + direction)
    return direction


def create_pose_stamped(position_x, position_y, rotation_z):
    goal = MoveBaseGoal()# Has to be created here
    q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, rotation_z)
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = position_x
    goal.target_pose.pose.position.y = position_y
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation.x = q_x
    goal.target_pose.pose.orientation.y = q_y
    goal.target_pose.pose.orientation.z = q_z
    goal.target_pose.pose.orientation.w = q_w
    return goal

   
def nav2goals():
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
 
    client.wait_for_server()
    
    goal_s = rospy.get_param("~goal_s")
    goal_r = rospy.get_param("~goal_r")
    goal_l = rospy.get_param("~goal_l")
    goal_t = rospy.get_param("~goal_t")
    img_topic = rospy.get_param("~img_topic")

    goal_pose_s = create_pose_stamped(goal_s['x'], goal_s['y'], radians(goal_s['w']))
    goal_pose_r = create_pose_stamped(goal_r['x'], goal_r['y'], radians(goal_r['w']))
    goal_pose_l = create_pose_stamped(goal_l['x'], goal_l['y'], radians(goal_l['w']))
    goal_pose_t = create_pose_stamped(goal_t['x'], goal_t['y'], radians(goal_t['w']))

    name_photo_s= photos_path + goal_s['photo_name']

    # First goal and Take photo signal right
    client.send_goal(goal_pose_s)
    wait = client.wait_for_result(rospy.Duration(50))
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        rospy.loginfo("Goal execution done!") 
        camera = TakePhoto(img_topic, name_photo_s)
        # Important! Allow up to one second for connection
        rospy.sleep(1)
        camera.save_picture(name_photo_s)

    # Process photo signal
    image = cv2.imread(name_photo_s)
    # traffic_signal = find_arrow_direction(image)
    traffic_signal = "test"
    traffic_signal = signal_detected(name_photo_s)

    rospy.sleep(5)
    print(traffic_signal)
    #traffic_signal = "right"
    while traffic_signal == "test":
        traffic_signal = signal_detected(name_photo_s)

    wait = client.wait_for_result(rospy.Duration(20))
    print(traffic_signal)
    if traffic_signal == "right":
        rospy.loginfo("Signal detected: RIGHT!")
        waypoints = [goal_pose_r, goal_pose_t]
    elif traffic_signal == "left":
        rospy.loginfo("Signal detection: LEFT!")
        waypoints = [goal_pose_l, goal_pose_t]
    else:
        print("why______________________")
        print(traffic_signal)
        waypoints = [goal_pose_s, goal_pose_t]
        rospy.loginfo("Not a correct detection!")

    # --- Follow Waypoints ---
    for i in range(2):
        client.send_goal(waypoints[i])
        wait = client.wait_for_result(rospy.Duration(50))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            rospy.loginfo("Goal execution done!")   

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_waypoints')
        # Initialize the ROS package manager
        rospack = rospkg.RosPack()
        # Get the path of the 'rubot_projects' package
        rubot_projects_path = rospack.get_path('rubot_projects')
        # Construct the full path to the photos directory
        photos_path = rubot_projects_path + '/photos/'

        np.set_printoptions(suppress=True)
        # Load the model
        model = load_model(rubot_projects_path + '/src/model/' + "keras_model.h5", compile=False)
        # Load the labels
        class_names = open(rubot_projects_path + '/src/model/' + "labels.txt", "r").readlines()

        nav2goals()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")