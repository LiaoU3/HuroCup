#! /usr/bin/env python
import rospy
import cv2

def main():
    rospy.init_node('target_tracker')
    rospy.loginfo('target_tracker activated !')

    pub_position = rospy.Publisher('/get_aruco_position', Pose2D, queue_size = 1)
    
    cap = cv2.VideoCapture('/dev/WebCamera')
    ret = cap.set(cv2.CAP_PROP_FRAME_WIDTH ,320)
    ret = cap.set(cv2.CAP_PROP_FRAME_HEIGHT,240) 

    position = Pose2D()

    while not rospy.is_shutdown():
        ret, frame = cap.read()

