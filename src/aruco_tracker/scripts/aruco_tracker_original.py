#! /usr/bin/env python
import numpy as np
import math
import cv2
from cv2 import aruco
import glob

import rospy
import rospkg
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Int16, Float32

if __name__ =='__main__':

    rospy.init_node('aruco_tracker')
    rospy.loginfo('aruco_tracker activated ! ')
    
    pub_position  = rospy.Publisher('/get_aruco_position' , Pose2D,  queue_size = 1)
    pub_size      = rospy.Publisher('/get_aruco_size'     , Int16 ,  queue_size = 1)
    cap = cv2.VideoCapture('/dev/NewCam')
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH , 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240) 

    position  = Pose2D()
    size      = -1
    
    # set dictionary size depending on the aruco marker selected
    aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
    parameters = aruco.DetectorParameters_create()

    ###------------------ ARUCO TRACKER ---------------------------
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        # operations on the frame
        if ret :
            # lists of ids and the corners belonging to each id

            corners, ids, _ = aruco.detectMarkers(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), aruco_dict, parameters=parameters)

            if ids is not None:
                for i in range(0, ids.size):
                    if ids[i,0] == 0:
                        # point to the center of aruco
                        M = cv2.moments(corners[i])
                        x = int(M["m10"] / M["m00"])
                        y = int(M["m01"] / M["m00"])
                        size = int(M["m00"])

                        # draw something on the frame
                        # cv2.circle(frame, (x, y), 5, (0, 255, 255), -1)
                        
                        position.x = x
                        position.y = y

                        break
                    else:
                        position.x = -1
                        position.y = -1
                        size       = -1
            else:
                position.x = -1
                position.y = -1
                size       = -1
            pub_position.publish(position)
            pub_size.publish(size)
            
        # display the resulting frame
        # cv2.imshow('frame',frame)
            
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()
    rospy.loginfo('aruco tracker terminated ! ')