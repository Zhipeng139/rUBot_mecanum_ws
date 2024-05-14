#!/usr/bin/env python3
import cv2
import numpy as np


def signal_detected(photo):
    image = cv2.imread(photo)
    cv2.imshow('Signal',image)
    signal = "left"
    return signal

def find_arrow_direction(image):
    # Convert to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

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

def find_ROI(image_path):
    src = cv2.imread(image_path)

    gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
    img = cv2.medianBlur(gray, 7)

    gaussian = cv2.GaussianBlur(img, (5, 5), 1.5)
    canny = cv2.Canny(gaussian, 150, 180)

    circles = cv2.HoughCircles(canny, cv2.HOUGH_GRADIENT, 1, 100, param1=100, param2=30, minRadius=0, maxRadius=600)

    if circles is not None:
        for x, y, r in circles[0]: 
            #cv2.circle(src, (int(x), int(y)), int(r), (0, 0, 255), 2, cv2.LINE_AA)
            if int(y-r) < 0 or int(x-r) < 0:
                crop_img = src[0:int(y+r), 0:int(x+r)]
            else:
                crop_img = src[int(y-r):int(y+r), int(x-r):int(x+r)]
    return crop_img    


if __name__ == '__main__':
    # Read signal
    image = cv2.imread('left.png')
    photo = "left.png"
    #signal = signal_detected(photo)
    #print("Siganl detected: ", signal)
    #cv2.imshow('Signal',image)
    #image_path = sys.argv[1]
    #roi = find_ROI(image_path)
    roi = photo
    direction = find_arrow_direction(roi)
    print("Direction:", direction)
    cv2.imshow('Signal',image)

    cv2.waitKey(0)
    cv2.destroyAllWindows()

    