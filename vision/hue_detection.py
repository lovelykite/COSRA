#!/usr/bin/env python
import numpy as np
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from cv_bridge.boost.cv_bridge_boost import getCvType


usb_cam = cv2.VideoCapture(0)

def hue_detect():
    image_pub = rospy.Publisher("/COSRA/hue_detection",Image, queue_size=10)
    rospy.init_node('hue_detection', anonymous=True)
    #rate = rospy.Rate(30) # 30hz
    bridge = CvBridge()
    while not rospy.is_shutdown():
        
        _, imageFrame = usb_cam.read() 
        
        hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV) 
        
        red_lower = np.array([136, 87, 111], np.uint8) 
        red_upper = np.array([180, 255, 255], np.uint8) 
        red_mask = cv2.inRange(hsvFrame, red_lower, red_upper) 
        
        green_lower = np.array([25, 52, 72], np.uint8) 
        green_upper = np.array([102, 255, 255], np.uint8) 
        green_mask = cv2.inRange(hsvFrame, green_lower, green_upper) 
        
        blue_lower = np.array([94, 80, 2], np.uint8) 
        blue_upper = np.array([120, 255, 255], np.uint8) 
        blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper) 
       
        kernal = np.ones((5, 5), "uint8") 

        # For red color 
        red_mask = cv2.dilate(red_mask, kernal) 
        res_red = cv2.bitwise_and(imageFrame, imageFrame, mask = red_mask) 
        # For green color 
        green_mask = cv2.dilate(green_mask, kernal) 
        res_green = cv2.bitwise_and(imageFrame, imageFrame, mask = green_mask) 
        # For blue color 
        blue_mask = cv2.dilate(blue_mask, kernal) 
        res_blue = cv2.bitwise_and(imageFrame, imageFrame, mask = blue_mask) 
        
        # Creating contour to track red color 
        _, contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
        for pic, contour in enumerate(contours): 
            area = cv2.contourArea(contour) 
            if(area > 300): 
                x, y, w, h = cv2.boundingRect(contour) 
                imageFrame = cv2.rectangle(imageFrame, (x, y), (x + w, y + h), (0, 0, 255), 2) 
                cv2.putText(imageFrame, "Red", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255)) 
                
        # Creating contour to track green color 
        _, contours, hierarchy = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
        for pic, contour in enumerate(contours): 
            area = cv2.contourArea(contour) 
            if(area > 300): 
                x, y, w, h = cv2.boundingRect(contour) 
                imageFrame = cv2.rectangle(imageFrame, (x, y), (x + w, y + h), (0, 255, 0), 2) 
                cv2.putText(imageFrame, "Green", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0)) 
                    
        # Creating contour to track blue color 
        _, contours, hierarchy = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
        for pic, contour in enumerate(contours): 
            area = cv2.contourArea(contour) 
            if(area > 300): 
                x, y, w, h = cv2.boundingRect(contour) 
                imageFrame = cv2.rectangle(imageFrame, (x, y), (x + w, y + h), (255, 0, 0), 2) 
                cv2.putText(imageFrame, "Blue", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0)) 

        cv2.imshow("Image window", imageFrame)
        cv2.waitKey(3)
        try:
            image_pub.publish(bridge.cv2_to_imgmsg(imageFrame, "bgr8"))
        except CvBridgeError as e:
            print(e)
        # rate.sleep()

if __name__ == '__main__':
    try:
        hue_detect()
    except rospy.ROSInterruptException:
        pass
