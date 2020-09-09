#! /usr/bin/env python
import numpy as np
import cv2
import rospy
import rospkg
import yaml
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Int16

# To show all the windows and trackbars (True or False)
show_win = False

rospy.init_node('marathon_line_detector')
rospy.loginfo('marathon_line_detector activated ! ')

pub_center = rospy.Publisher('/get_line_center', Pose2D, queue_size = 1)
# pub_count  = rospy.Publisher('/get_pixel_count', Int16,  queue_size = 1)
center = Pose2D()

def nothing(x):
    pass

rospack = rospkg.RosPack()

with open(rospack.get_path('marathon_line_detector') + '/src/color_range.yaml', 'r') as f:
    data = yaml.load(f)
    color1_lower_1 = np.array(data[0]['color1']['lower_1'])
    color1_lower_2 = np.array(data[0]['color1']['lower_2'])
    color1_upper_1 = np.array(data[0]['color1']['upper_1'])
    color1_upper_2 = np.array(data[0]['color1']['upper_2'])
    color2_lower   = np.array(data[1]['color2']['lower'])
    color2_upper   = np.array(data[1]['color2']['upper'])


cap = cv2.VideoCapture(0)

cap.set(cv2.CAP_PROP_FRAME_WIDTH ,320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,240) 
if show_win :
    winname = 'original_frame'
    cv2.namedWindow(winname)

    # First Color Detection

    cv2.createTrackbar('Color1_H_lower_1', winname, 0, 180,nothing)
    cv2.setTrackbarPos('Color1_H_lower_1', winname, color1_lower_1[0])

    cv2.createTrackbar('Color1_H_upper_1', winname, 0, 180,nothing)
    cv2.setTrackbarPos('Color1_H_upper_1', winname, color1_upper_1[0])

    cv2.createTrackbar('Color1_H_lower_2', winname, 0, 180,nothing)
    cv2.setTrackbarPos('Color1_H_lower_2', winname, color1_lower_2[0])

    cv2.createTrackbar('Color1_H_upper_2', winname, 0, 180,nothing)
    cv2.setTrackbarPos('Color1_H_upper_2', winname, color1_upper_2[0])

    cv2.createTrackbar('Color1_S_lower'  , winname, 0, 255,nothing)
    cv2.setTrackbarPos('Color1_S_lower'  , winname, color1_lower_1[1])

    cv2.createTrackbar('Color1_S_upper'  , winname, 0, 255,nothing)
    cv2.setTrackbarPos('Color1_S_upper'  , winname, color1_upper_1[1])

    cv2.createTrackbar('Color1_V_lower'  , winname, 0, 255,nothing)
    cv2.setTrackbarPos('Color1_V_lower'  , winname, color1_lower_1[2])

    cv2.createTrackbar('Color1_V_upper'  , winname, 0, 255,nothing)
    cv2.setTrackbarPos('Color1_V_upper'  , winname, color1_upper_1[2])

    # Second Color Detection

    cv2.createTrackbar('Color2_H_lower' , winname, 0, 180,nothing)
    cv2.setTrackbarPos('Color2_H_lower' , winname, color2_lower[0])

    cv2.createTrackbar('Color2_H_upper' , winname, 0, 180,nothing)
    cv2.setTrackbarPos('Color2_H_upper' , winname, color2_upper[0])

    cv2.createTrackbar('Color2_S_lower'  , winname, 0, 255,nothing)
    cv2.setTrackbarPos('Color2_S_lower'  , winname, color2_lower[1])

    cv2.createTrackbar('Color2_S_upper'  , winname, 0, 255,nothing)
    cv2.setTrackbarPos('Color2_S_upper'  , winname, color2_upper[1])

    cv2.createTrackbar('Color2_V_lower'  , winname, 0, 255,nothing)
    cv2.setTrackbarPos('Color2_V_lower'  , winname, color2_lower[2])

    cv2.createTrackbar('Color2_V_upper'  , winname, 0, 255,nothing)
    cv2.setTrackbarPos('Color2_V_upper'  , winname, color2_upper[2])



while not rospy.is_shutdown():
    ret, frame = cap.read()

    if ret :
        # First Color----------------------------------------------------------------
        blurred = cv2.GaussianBlur(frame, (29, 29), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        if show_win :
            color1_h_lower_1 = cv2.getTrackbarPos('Color1_H_lower_1', winname)
            color1_h_upper_1 = cv2.getTrackbarPos('Color1_H_upper_1', winname)
            color1_h_lower_2 = cv2.getTrackbarPos('Color1_H_lower_2', winname)
            color1_h_upper_2 = cv2.getTrackbarPos('Color1_H_upper_2', winname)

            color1_s_lower = cv2.getTrackbarPos('Color1_S_lower', winname)
            color1_s_upper = cv2.getTrackbarPos('Color1_S_upper', winname)

            color1_v_lower = cv2.getTrackbarPos('Color1_V_lower', winname)
            color1_v_upper = cv2.getTrackbarPos('Color1_V_upper', winname)

            color1_lower_1 = np.array([color1_h_lower_1, color1_s_lower, color1_v_lower])
            color1_lower_2 = np.array([color1_h_lower_2, color1_s_lower, color1_v_lower])
            color1_upper_1 = np.array([color1_h_upper_1, color1_s_upper, color1_v_upper])
            color1_upper_2 = np.array([color1_h_upper_2, color1_s_upper, color1_v_upper])

        
        color1_mask_1 = cv2.inRange(hsv, color1_lower_1, color1_upper_1)
        color1_mask_2 = cv2.inRange(hsv, color1_lower_2, color1_upper_2)            
        color1_mask = color1_mask_1 | color1_mask_2

        if show_win :
            _, color1_contours, _ = cv2.findContours(color1_mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

            # cv2.drawContours(frame,color1_contours,-1,(0,0,255),3)

        # ---------------------------------------------------------------------------

        # Second Color---------------------------------------------------------------
        if show_win :
            color2_h_lower = cv2.getTrackbarPos('Color2_H_lower', winname)
            color2_h_upper = cv2.getTrackbarPos('Color2_H_upper', winname)

            color2_s_lower = cv2.getTrackbarPos('Color2_S_lower', winname)
            color2_s_upper = cv2.getTrackbarPos('Color2_S_upper', winname)

            color2_v_lower = cv2.getTrackbarPos('Color2_V_lower', winname)
            color2_v_upper = cv2.getTrackbarPos('Color2_V_upper', winname)

            color2_lower   = np.array([color2_h_lower, color2_s_lower, color2_v_lower])
            color2_upper   = np.array([color2_h_upper, color2_s_upper, color2_v_upper])

        color2_mask = cv2.inRange(hsv, color2_lower, color2_upper)

        if show_win :
            _, color2_contours, _ = cv2.findContours(color2_mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

            # cv2.drawContours(frame,color2_contours,-1,(0,255,0),3)

        # --------------------------------------------------------------------------
        final_mask = color1_mask | color2_mask
        _, final_countours, _ = cv2.findContours(final_mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        cX = cY = -1 
        if len(final_countours) > 0:
            cnt = max(final_countours, key = cv2.contourArea)
            cntr_area = cv2.contourArea(cnt)
            box_x, box_y, box_w, box_h = cv2.boundingRect(cnt)
            cX = box_x + box_w / 2
            cY = box_y + box_h / 2
            if show_win:
                cv2.circle(frame, (cX, cY), 5, (1, 227, 254), -1)
                cv2.rectangle(frame, (box_x, box_y), (box_x + box_w, box_y + box_h), (0, 255, 0), 2)

        # pub_count.publish(int(np.sum(final_mask) // 255))

        # M = cv2.moments(final_mask)
        # if M['m00'] > 0 :
        #     cX = int(M['m10'] / M['m00'])
        #     cY = int(M['m01'] / M['m00'])
        # else :
        #     cX = -1
        #     cY = -1
        center.x = cX
        center.y = cY

        pub_center.publish(center)

        if show_win :
            cv2.imshow(winname, frame)
            cv2.imshow('mask', final_mask)

    k = cv2.waitKey(1) & 0xFF
    if k== ord('q'):
        break
    elif k == ord('s'):
        with open(rospack.get_path('marathon_line_detector') + '/src/color_range.yaml', 'w') as f :
            dp = [{'color1': {'lower_1': [color1_h_lower_1, color1_s_lower, color1_v_lower], 'lower_2': [color1_h_lower_2, color1_s_lower, color1_v_lower], 'upper_2': [color1_h_upper_2, color1_s_upper, color1_v_upper], 'upper_1': [color1_h_upper_1, color1_s_upper, color1_v_upper]}}, {'color2': {'upper': [color2_h_upper, color2_s_upper, color2_v_upper], 'lower': [color2_h_lower, color2_s_lower, color2_v_lower]}}]
            yaml.dump(dp, f)

cap.release()
cv2.destroyAllWindows()
rospy.loginfo('marathon_line_detector terminated ! ')
