#! /usr/bin/env/python
import cv2
import numpy as np
import time

class spots:
    loc = 0

def get_center(rois):
    X = np.array([[1,0],[0,1],[0.5,0],[0,0.5]])
    return np.around(rois.dot(X))

def set_goal(color_goal, rois):
    center = get_center(rois).T
    color_goal[1:3,] = color_goal[1:3,]*center
    color_goal[3:,] = color_goal[3:,]*set_Z
    return color_goal

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

    #############################################################
    
    red_lower = np.array([124, 66, 136], np.uint8) 
    red_upper = np.array([179, 178, 255], np.uint8) 
    red_mask = cv2.inRange(hsvFrame, red_lower, red_upper) 
    
    green_lower = np.array([83, 95, 109], np.uint8) 
    green_upper = np.array([88, 246, 255], np.uint8) 
    green_mask = cv2.inRange(hsvFrame, green_lower, green_upper) 
    
    blue_lower = np.array([95, 198, 212], np.uint8) 
    blue_upper = np.array([103, 255, 255], np.uint8) 
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

def callback(foo):
    pass

def sorting_box(color_goal, frame):
    if np.any(color_goal[0, :2] == 2) or np.any(color_goal[0, :2] == 3) or np.any(color_goal[0, 2:4] == 1) or np.any(color_goal[0, 2:4] == 3) or np.any(color_goal[0, 4:] == 1) or np.any(color_goal[0, 4:] == 2):
        try:
            empty = np.where(color_goal[0, :] == 0)
            print(empty[0][0])
            if empty[0][0] == 0 or empty[0][0] == 1:
                try:
                    wrong = np.where(color_goal[0, :] == 1)
                    cv2.putText(frame, 'Move RED', (310, 30), font, 1, (0, 0, 255), 3)
                    cv2.imshow('frame', frame)
                    print("Move RED")
                except:
                    wrong = np.where(color_goal[0, :] != 0 & color_goal[0, :] != 1)
                    cv2.putText(frame, 'Move Other Color', (310, 30), font, 1, (0, 0, 0), 3)
                    cv2.imshow('frame', frame)
                    print("Move Other Color")
            elif empty[0][0] == 2 or empty[0][0] == 3:
                try:
                    wrong = np.where(color_goal[0, :] == 2)
                    cv2.putText(frame, 'Move GREEN', (310, 30), font, 1, (0, 255, 0), 3)
                    cv2.imshow('frame', frame)
                    print("Move GREEN")
                except:
                    wrong = np.where(color_goal[0, :] != 0 & color_goal[0, :] != 2)
                    cv2.putText(frame, 'Move Other Color', (310, 30), font, 1, (0, 0, 0), 3)
                    cv2.imshow('frame', frame)
                    print("Move Other Color")
            else:
                try:
                    wrong = np.where(color_goal[0, :] == 3)
                    cv2.putText(frame, 'Move BLUE', (310, 30), font, 1, (255, 0, 0), 3)
                    cv2.imshow('frame', frame)
                    print("Move BLUE")
                except:
                    wrong = np.where(color_goal[0, :] != 0 & color_goal[0, :] != 3)
                    cv2.putText(frame, 'Move Other Color', (310, 30), font, 1, (0, 0, 0), 3)
                    cv2.imshow('frame', frame)
                    print("Move Other Color")   
            start = color_goal[1:, wrong[0][0]].T
            goal = color_goal[1:, empty[0][0]].T

            print("start :", start)
            print("goal :", goal)
            return start, goal
        except:
            cv2.putText(frame, 'Make an empty space!', (310, 30), font, 1, (0, 0, 0), 3)
            cv2.imshow('frame', frame)
            print("Make an empty space!")        

    else:
        cv2.putText(frame, 'Complete!', (310, 30), font, 1, (0, 0, 0), 3)
        cv2.imshow('frame', frame)
        print ("Sorting Box Complete!")
        
# find parameters with Trackbar
# cv2.namedWindow('parameters')
# cv2.createTrackbar('Threshold1', 'parameters', 186, 700, callback)
# cv2.createTrackbar('Threshold2', 'parameters', 122, 700, callback)
# cv2.createTrackbar('Min pixels', 'parameters', 100, 1500, callback)
# cv2.createTrackbar('Max pixels', 'parameters', 323, 1500, callback)

rois = np.array([[205,125,80,80],
        [205,220,80,80],
        [301,125,80,80],
        [301,220,80,80],
        [398,125,80,80],
        [398,220,80,80]])
# rois = [[int(float(j)) for j in i] for i in rois]
color_goal = np.vstack((np.zeros(6),np.ones((3,6))))
set_Z = 5

color_goal = set_goal(color_goal, rois)
# Roi numbering
#          B  G  R
#########  1  3  5  #########
#########  2  4  6  #########

# define color_goal array
# color state (None : 0, R :1, G : 2, B : 3)
# goal_x
# goal_y
# goal_z

VIDEO_SOURCE = 0
cap = cv2.VideoCapture(VIDEO_SOURCE)

while True:
    
    start = time.time() 

    spots.loc = 0

    ret, frame = cap.read()
    ret2, frame2 = cap.read()

    #set parameters
    # min = cv2.getTrackbarPos('Min pixels', 'parameters')
    # max = cv2.getTrackbarPos('Max pixels', 'parameters')
    # lowThreshold = cv2.getTrackbarPos('Threshold1', 'parameters')
    # highThreshold = cv2.getTrackbarPos('Threshold2', 'parameters')
    min = 700
    max = 1500
    lowThreshold = 186
    highThreshold = 122

    for i in range(len(rois)):
        drawRectangle(frame, rois[i][0], rois[i][1], rois[i][2], rois[i][3], i)

    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(frame, 'Available spots: ' + str(spots.loc), (10, 30), font, 1, (0, 235, 255), 2)
    cv2.imshow('frame', frame)

    canny = cv2.Canny(frame2, lowThreshold, highThreshold)
    cv2.imshow('canny', canny)

    print (color_goal)

    #if robot = home state일때만 sorting_box 코드 실행하도록 할 예정
    #robot 제어 코드에서 home_flag topic을 보내주면 이를 수신하면 됨
    sorting_box(color_goal, frame)

    print("time :", time.time() - start)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
