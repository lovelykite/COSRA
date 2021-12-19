#! /usr/bin/env/python
import cv2
import numpy as np
import time

import rospy
from std_msgs.msg import Int32, Int32MultiArray
import threading

class spots:
    loc = 0

def drawRectangle(img, a, b, c, d, i):
    sub_img = img[b:b + d, a:a + c]
    edges = cv2.Canny(sub_img, lowThreshold, highThreshold)
    pix = cv2.countNonZero(edges)
    if pix in range(min, max):
        cv2.rectangle(img, (a, b), (a + c, b + d), (0, 255, 0), 3) #draw green rectangle
        spots.loc += 1
        color_goal[0][i] = 0
    else:
        hue_detect(sub_img, i)
        cv2.rectangle(img, (a, b), (a + c, b + d), (0, 0, 255), 3) #draw red rectangle

def hue_detect(sub_img, i):
           
    imageFrame = sub_img
    
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV) 

    ####################Under the Class Light#########################
    
    # red_lower = np.array([124, 66, 136], np.uint8) 
    # red_upper = np.array([179, 178, 255], np.uint8) 
    # red_mask = cv2.inRange(hsvFrame, red_lower, red_upper) 
    
    # green_lower = np.array([83, 95, 109], np.uint8) 
    # green_upper = np.array([88, 246, 255], np.uint8) 
    # green_mask = cv2.inRange(hsvFrame, green_lower, green_upper) 
    
    # blue_lower = np.array([95, 198, 212], np.uint8) 
    # blue_upper = np.array([103, 255, 255], np.uint8) 
    # blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper) 

    #####################For the Test##########################
    
    red_lower = np.array([0, 117, 104], np.uint8) 
    red_upper = np.array([2, 255, 176], np.uint8) 
    red_mask = cv2.inRange(hsvFrame, red_lower, red_upper) 
    
    green_lower = np.array([60, 108, 67], np.uint8) 
    green_upper = np.array([84, 255, 161], np.uint8) 
    green_mask = cv2.inRange(hsvFrame, green_lower, green_upper) 
    
    blue_lower = np.array([96, 204, 120], np.uint8) 
    blue_upper = np.array([102, 255, 211], np.uint8) 
    blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper) 

    #############################################################
        
    kernal = np.ones((5, 5), "uint8") 

    red_mask = cv2.dilate(red_mask, kernal) 
    res_red = cv2.bitwise_and(imageFrame, imageFrame, mask = red_mask) 
    green_mask = cv2.dilate(green_mask, kernal) 
    res_green = cv2.bitwise_and(imageFrame, imageFrame, mask = green_mask) 
    blue_mask = cv2.dilate(blue_mask, kernal) 
    res_blue = cv2.bitwise_and(imageFrame, imageFrame, mask = blue_mask) 
    
    contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
    for pic, contour in enumerate(contours): 
        area = cv2.contourArea(contour) 
        color_goal[0][i] = 1
        if(area > 300): 
            x, y, w, h = cv2.boundingRect(contour) 
            imageFrame = cv2.rectangle(imageFrame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            
    contours, hierarchy = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
    for pic, contour in enumerate(contours): 
        area = cv2.contourArea(contour) 
        color_goal[0][i] = 2
        if(area > 300): 
            x, y, w, h = cv2.boundingRect(contour) 
            imageFrame = cv2.rectangle(imageFrame, (x, y), (x + w, y + h), (0, 255, 0), 2) 
                
    contours, hierarchy = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
    for pic, contour in enumerate(contours): 
        area = cv2.contourArea(contour) 
        color_goal[0][i] = 3
        if(area > 300): 
            x, y, w, h = cv2.boundingRect(contour) 
            imageFrame = cv2.rectangle(imageFrame, (x, y), (x + w, y + h), (255, 0, 0), 2) 

#sorting_box algorithm
def sorting_box(color_goal, frame):
    #find wrong box
    if np.any(color_goal[0, :2] == 2) or np.any(color_goal[0, :2] == 3) or np.any(color_goal[0, 2:4] == 1) or np.any(color_goal[0, 2:4] == 3) or np.any(color_goal[0, 4:] == 1) or np.any(color_goal[0, 4:] == 2):
        try:
            empty = np.where(color_goal[0, :] == 0)
            print(empty[0][0])
            temp = color_goal[0,:]
            print(temp)
            if empty[0][0] == 0 or empty[0][0] == 1:
                if (color_goal[0, 2:] == 1).any() == True:
                    temp[0:2] = 0
                    wrong = np.where(temp[:] == 1)
                    cv2.putText(frame, 'Move RED', (310, 30), font, 1, (0, 0, 255), 3)
                    # cv2.imshow('frame', frame)
                    cv2.imshow('color', frame)
                    print("Move RED")
                    start = color_goal[1, wrong[0][0]]
                else:
                    wrong = np.where((color_goal[0, :] != 0) & (color_goal[0, :] != 1))
                    cv2.putText(frame, 'Move Other Color', (310, 30), font, 1, (0, 0, 0), 3)
                    # cv2.imshow('frame', frame)
                    cv2.imshow('color', frame)
                    print("Move Other Color")
                    start = color_goal[1, np.random.choice(wrong[0][:], 1)]
            elif empty[0][0] == 2 or empty[0][0] == 3:
                if (color_goal[0, 0:2] == 2).any() == True or (color_goal[0, 4:] == 2).any() == True:
                    temp[2:4] = 0
                    wrong = np.where((temp[:] == 2) | (temp[:] == 2))
                    cv2.putText(frame, 'Move GREEN', (310, 30), font, 1, (0, 255, 0), 3)
                    # cv2.imshow('frame', frame)
                    cv2.imshow('color', frame)
                    print("Move GREEN")
                    start = color_goal[1, wrong[0][0]]
                else:
                    wrong = np.where((color_goal[0, :] != 0) & (color_goal[0, :] != 2))
                    cv2.putText(frame, 'Move Other Color', (310, 30), font, 1, (0, 0, 0), 3)
                    # cv2.imshow('frame', frame)
                    cv2.imshow('color', frame)
                    print("Move Other Color")
                    start = color_goal[1, np.random.choice(wrong[0][:], 1)]
            else:
                if (color_goal[0, :4] == 3).any() == True:
                    temp[4:] = 0
                    wrong = np.where(temp[:] == 3)
                    cv2.putText(frame, 'Move BLUE', (310, 30), font, 1, (255, 0, 0), 3)
                    # cv2.imshow('frame', frame)
                    cv2.imshow('color', frame)
                    print("Move BLUE")
                    start = color_goal[1, wrong[0][0]]
                else:
                    wrong = np.where((color_goal[0, :] != 0) & (color_goal[0, :] != 3))
                    cv2.putText(frame, 'Move Other Color', (310, 30), font, 1, (0, 0, 0), 3)
                    # cv2.imshow('frame', frame)
                    cv2.imshow('color', frame)
                    print("Move Other Color") 
                    start = color_goal[1, np.random.choice(wrong[0][:], 1)]
            
            goal = color_goal[1, empty[0][0]]

            print("start :", start)
            print("goal :", goal)
            send_joints(start, goal)
            return start, goal
        except:
            cv2.putText(frame, 'Make an empty space!', (310, 30), font, 1, (0, 0, 0), 3)
            # cv2.imshow('frame', frame)
            cv2.imshow('color', frame)
            print("Make an empty space!")        

    else:
        #if all boxes are in right place
        cv2.putText(frame, 'Complete!', (310, 30), font, 1, (0, 0, 0), 3)
        # cv2.imshow('frame', frame)
        cv2.imshow('color', frame)
        print ("Sorting Box Complete!")
        #sorting box ceremony
        start = 7
        goal = 8
        send_joints(start, goal)
        return start, goal
        
# find parameters with Trackbar
# cv2.namedWindow('parameters')
# cv2.createTrackbar('Threshold1', 'parameters', 186, 700, callback)
# cv2.createTrackbar('Threshold2', 'parameters', 122, 700, callback)
# cv2.createTrackbar('Min pixels', 'parameters', 100, 1500, callback)
# cv2.createTrackbar('Max pixels', 'parameters', 323, 1500, callback)

#ser rois
rois = np.array([[382,189,85,85],
        [380,17,80,80],
        [269,195,85,85],
        [272,14,80,80],
        [155,191,85,85],
        [161,14,80,80]])
# rois = [[int(float(j)) for j in i] for i in rois]
color_goal = np.vstack((np.zeros(6),np.array([1, 2, 3, 4, 5, 6])))

# Roi numbering
#          B  G  R
#########  1  3  5  #########
#########  2  4  6  #########

# define color_goal array
# color state (None : 0, R :1, G : 2, B : 3)
# goal_x
# goal_y
# goal_z


def callback_flag(data):
    global flag
    flag = data.data


def listen_flag():
    rospy.Subscriber("home_flag", Int32, callback_flag)
    rospy.spin()

def send_joints(start, goal):
    try: 
        pub = rospy.Publisher('send_goal', Int32MultiArray, queue_size=10)    
        joints = Int32MultiArray()
        joints.data = list(map(int, (np.hstack((start, goal)))))
        pub.publish(joints)
        print(joints)
    except:
        pass


VIDEO_SOURCE = 0
cap = cv2.VideoCapture(VIDEO_SOURCE)

flag = 0
send_joint = np.zeros(6)

if __name__ == '__main__':       

    while True: 
        rospy.init_node('vision', anonymous=True)
        #listen flag ROS topic from control node
        listener = threading.Thread(target = listen_flag)
        listener.start()

        spots.loc = 0

        ret, frame = cap.read()
        ret2, frame2 = cap.read()

        #set parameters
        # min = cv2.getTrackbarPos('Min pixels', 'parameters')
        # max = cv2.getTrackbarPos('Max pixels', 'parameters')
        # lowThreshold = cv2.getTrackbarPos('Threshold1', 'parameters')
        # highThreshold = cv2.getTrackbarPos('Threshold2', 'parameters')
        min = 500
        max = 1600
        lowThreshold = 186
        highThreshold = 122

        for i in range(len(rois)):
            drawRectangle(frame, rois[i][0], rois[i][1], rois[i][2], rois[i][3], i)

        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(frame, 'Available spots: ' + str(spots.loc), (10, 30), font, 1, (0, 235, 255), 2)
        cv2.namedWindow('color', cv2.WINDOW_NORMAL)
        cv2.imshow('color', frame)
        cv2.resizeWindow('color', 1000, 800)  

        canny = cv2.Canny(frame2, lowThreshold, highThreshold)
        cv2.namedWindow('canny', cv2.WINDOW_NORMAL)
        cv2.imshow('canny', canny)
        cv2.resizeWindow('canny', 1000, 800)  
        

        # print (color_goal)
        if flag == 0:
            sorting_box(color_goal, frame)
            print(color_goal)
            # send_joints(start_p, goal_p)          
       
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
