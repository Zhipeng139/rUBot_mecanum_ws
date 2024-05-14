#!/usr/bin/env python3
import rospy
import cv2 as cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


def signal_detected(photo):
    image = cv.imread(photo)
    cv.imshow('Signal',image)
    signal = "left"
    return signal

def find_arrow_direction(image):
    # Convert to grayscale
    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

    # Apply edge detection
    gaussian = cv.GaussianBlur(gray, (3, 3), 1)
    edges = cv.Canny(gaussian, 150, 180)
    #plt.imshow(edges, cmap='gray')
    #plt.show()

    # Find contours
    contours, _ = cv.findContours(edges, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    # Assume the largest contour is the arrow
    arrow_contour = max(contours, key=cv.contourArea)

    # Find the bounding rectangle for the largest contour
    x, y, w, h = cv.boundingRect(arrow_contour)
    
    # Draw the bounding rectangle
    #cv.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
    #plt.imshow(image)
    #plt.show()

    # Calculate the center of the arrow
    center = (x + w // 2, y + h // 2)

    # Determine direction based on the geometry of the arrow
    # Find the center of mass of the arrow
    M = cv.moments(arrow_contour)
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

def find_ROI(image_path):
    src = cv.imread(image_path)

    gray = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
    img = cv.medianBlur(gray, 7)

    gaussian = cv.GaussianBlur(img, (5, 5), 1.5)
    canny = cv.Canny(gaussian, 150, 180)

    circles = cv.HoughCircles(canny, cv.HOUGH_GRADIENT, 1, 100, param1=100, param2=30, minRadius=0, maxRadius=600)
    crop_img = None
    if circles is not None:
        for x, y, r in circles[0]: 
            #cv.circle(src, (int(x), int(y)), int(r), (0, 0, 255), 2, cv.LINE_AA)
            if int(y-r) < 0 or int(x-r) < 0:
                crop_img = src[0:int(y+r), 0:int(x+r)]
            else:
                crop_img = src[int(y-r):int(y+r), int(x-r):int(x+r)]
    return crop_img    

class TakePhoto:
    def __init__(self, img_topic, image_title):
        self.bridge = CvBridge()
        self.image_received = False
        self.cv_image = None

        # Connect image topic
        self.image_sub = rospy.Subscriber(img_topic, Image, self.callback)

    def callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv.putText(self.cv_image, "Image 1", (100, 290), cv.FONT_HERSHEY_TRIPLEX, 1, (0, 255, 255), 1)
            self.image_received = True
        except CvBridgeError as e:
            rospy.logerr(e)

    def save_picture(self, img_title):
        if self.image_received:
            cv.imwrite(img_title, self.cv_image)
            rospy.loginfo("Saved image " + img_title)
        else:
            rospy.loginfo("No images received")

if __name__ == '__main__':
    # Initialize
    rospy.init_node('take_photo', anonymous=False)

    # Get parameters from launch file
    img_topic = rospy.get_param('~image_topic', '/usb_cam/image_raw')
    img_title = rospy.get_param('~image_title', './src/rubot_projects/photos/photo3_hw_2.jpg')

    # Create TakePhoto instance
    camera = TakePhoto(img_topic, img_title)
    # Allow up to one second for connection
    rospy.sleep(1)
    camera.save_picture(img_title)

    img = find_ROI(img_title)
    direction = find_arrow_direction(img)
    print(direction)
    # Keep the node running
    rospy.spin()

