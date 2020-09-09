#! /usr/bin/env python
import os
import rospy
import numpy
import socket
# import serial
import subprocess

from time import sleep
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32, Float32, Bool, Int32MultiArray
# from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger, TriggerResponse
from robot_msgs.msg import HeadStatus

UDP_IP = "127.0.0.1"
MOTION_PORT = 8080
HEAD_PORT = 8081
BUTTON_PORT = 8083
comp_port   = 8082

motion_state = "init"

kill = "killall screen;"
cmd = "cd ~/UPennalizers-master/Player; screen -S dcm lua run_dcm.lua; screen -S player lua walk_server.lua;"

gripperState = False

def velocity_callback(msg):
	rospy.loginfo("Message Received")
	rospy.loginfo(msg)
	global sock_motion
	MESSAGE = "walk " + str(msg.linear.x) + " " + str(msg.linear.y) + " " + str(msg.linear.z)
	# rospy.loginfo("Message to Darwin Motion Controller")
	# rospy.loginfo(MESSAGE)
	sock_motion.sendto(MESSAGE, (UDP_IP, MOTION_PORT))

def motion_state_callback(msg):
	rospy.loginfo("Motion State Received")
	rospy.loginfo(msg)
	global sock_motion
	MESSAGE = str(msg.data)
	# rospy.loginfo("Message to Darwin Motion Controller")
	# rospy.loginfo(MESSAGE)
	sock_motion.sendto(MESSAGE, (UDP_IP, MOTION_PORT))

# def gripper_state_callback(msg):
# 	rospy.loginfo("Gripper State Received")
# 	rospy.loginfo(msg)
# 	global sock_motion
# 	gripperState = msg.data
# 	if gripperState == True:
# 		MESSAGE = "grip 1"
# 	else:
# 		MESSAGE = "grip 0"
# 	rospy.loginfo("Message to Darwin Motion Controller")
# 	rospy.loginfo(MESSAGE)
# 	sock_motion.sendto(MESSAGE, (UDP_IP, MOTION_PORT))

def head_callback(pos_msg):
	# rospy.loginfo("Head Received")
	# rospy.loginfo(pos_msg)
	global sock_motion
	# headPan = pos_msg.data[0]
	# headTilt = pos_msg.data[1]
	headPan = pos_msg.pan
	headTilt = pos_msg.tilt
	# if headPan <= 1.401 and headPan >= -1.401 and headTilt < 0.001 and headTilt > -1.101 :
	MESSAGE = "head " + str(headPan)+" " + str(headTilt)
	# 	# rospy.loginfo("Message to Darwin Motion Controller")
	# 	# rospy.loginfo(MESSAGE)
	sock_motion.sendto(MESSAGE, (UDP_IP, HEAD_PORT))
	# else:
	# 	rospy.logwarn('Wrong pan or tilt (-1.1 < tilt and -1.4 < pan < 1.4 )')

def trigger_response(request):
    return TriggerResponse(success=True, message="controller run")

def sock_motion_Button():
	global sock_button
	rawData = sock_button.recv(1024)
	state = rawData.split(" ")
	buttonState = Int32MultiArray()
	buttonState.data = [int(state[0]), int(state[1])]
	button_pub.publish(buttonState)		

def kill_node():
	global sock_motion, sock_button
	sock_motion.close()
	# sock_button.close()
	rospy.signal_shutdown("shutdown time.") 

def foot_comp_callback(msg):
	MESSAGE = "comp {} ".format(round(msg.data,4))
	# MESSAGE = MESSAGE.encode() # python3
	sock_motion.sendto(MESSAGE, (UDP_IP, comp_port))

def main():
	rospy.init_node("darwin_motion_bridge")	

	os.system(kill)
	os.system(cmd)

	global sock_motion, sock_button
	sock_motion = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	sock_button = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	sock_button.bind((UDP_IP, BUTTON_PORT))

	# rospy.init_node("darwin_motion_bridge")	
	rospy.Subscriber("/motion/cmd_vel", Twist, velocity_callback)
	rospy.Subscriber("/motion/state", String, motion_state_callback)
	# gripper_state_subscriber = rospy.Subscriber("/gripper/state", Bool, gripper_state_callback)
	rospy.Subscriber("/head/pos", HeadStatus, head_callback)
	rospy.Subscriber("/walk/footcomp",  Float32, foot_comp_callback)
	# my_service = rospy.Service('/srv_controller', Trigger, trigger_response)
	global button_pub
	button_pub = rospy.Publisher("/button/state", Int32MultiArray, queue_size=1)

	while not rospy.is_shutdown():
		sock_motion_Button()		
	# rospy.loginfo("Spinning")
	rospy.spin()
	rospy.on_shutdown(kill_node)
	os.system(kill)

if __name__ == "__main__":
    main()
