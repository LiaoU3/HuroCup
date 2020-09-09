#! /usr/bin/env python
import math
import time
import rospy
import roslib
import numpy as np
from geometry_msgs.msg import Pose2D, Twist
from std_msgs.msg import String, Int16, Int32MultiArray,Float32
from robot_msgs.msg import HeadStatus

class SprintPlayer:
    def __init__(self):
        rospy.init_node('sprint_player')
        rospy.loginfo('sprint_player activated ! ')

        # position of the aruco marker
        self.marker_x    = -1
        self.marker_y    = -1
        self.marker_size = -1

        # state about velocity
        self.vel_x     = 0
        self.vel_y     = 0
        self.vel_angle = 0
        # I set this value becaust it might spin by itself
        self.vel_angle_offset = 0.025 # 0.06

        # limit of vel_y
        self.max_vel_y = 0.015
        # self.error_body_y = 0

        # limit of vel_angle
        self.max_vel_angle = 0.4

        # state about body
        self.state = String()
        self.state = "initial"

        # position about head
        self.pos_pan  =  0.0
        self.pos_tilt = -1.6

        # the limit of pan an d tilt
        self.pan_min  = -1.6
        self.pan_max  =  1.6
        self.tilt_min = -1.5
        self.tilt_max = -0.9

        # cout loss marker
        self.count_marker_loss = 0
        self.max_marker_size = 10000

        self.tilt_step = 0.01

        # camera
        self.frame_width  = 320 #rospy.get_param('/frame_width')
        self.frame_height = 240 #rospy.get_param('/frame_height')

        
        # rate
        self.freq = 60
        self.rate = rospy.Rate(self.freq)

        # backward x offset
        self.bwd_xcomp = True
        
        self.sum_err_pan  = 0
        self.sum_err_tilt = 0
        self.last_error_x = 0
        self.last_error_y = 0

        self.KP_pan  = 0.048 #0.04
        self.KI_pan  = 0.09
        self.KD_pan  = 0.006

        self.KP_tilt = 0.06 #0.04
        self.KI_tilt = 0.0
        self.KD_tilt = 0.0

        self.dt      = 1.0 / float(self.freq)
        
        self.button = [0, 0]
        self.previous_button = [0, 0]

        # publisher
        self.motion_vel_pub   = rospy.Publisher("/motion/cmd_vel", Twist,      queue_size=1)
        self.motion_state_pub = rospy.Publisher("/motion/state",   String,     queue_size=1)
        self.head_pub         = rospy.Publisher("/head/pos",       HeadStatus, queue_size=1)
        self.xcomp_pub        = rospy.Publisher("/walk/footcomp",  Float32,    queue_size=1)

        # subscriber
        rospy.Subscriber('/get_aruco_position', Pose2D, self.marker_pos_callback)
        rospy.Subscriber('/get_aruco_size'    , Int16 , self.marker_size_callback)
        # rospy.Subscriber('get_aruco_direction', Float32, self.marker_direction_callback)
        rospy.Subscriber("/button/state", Int32MultiArray, self.button_callback)

    def marker_pos_callback(self, pos_msg):
        self.marker_x = pos_msg.x
        self.marker_y = pos_msg.y

    def marker_size_callback(self, Int16):
        self.marker_size = Int16.data

    def marker_lost(self, threshold):
        if self.marker_x == -1:		
            self.count_marker_loss += 1
            if self.count_marker_loss >= threshold :
                return True
        else :
            self.count_marker_loss = 0
            return False

    def button_callback(self, pos_msg):
        
        self.button[0] = pos_msg.data[0]
        self.button[1] = pos_msg.data[1]

    def walk(self, x, y, angle):
        self.vel_x = x
        self.vel_y = y
        self.vel_angle = angle

        velocity = Twist()
        velocity.linear.x = x
        velocity.linear.y = y
        velocity.linear.z = angle
        self.motion_vel_pub.publish(velocity)

    def head_move(self, pan, tilt):
        # self.pos_pan, self.pos_tilt =  self.head_limit(pan, tilt)
        head_pos = HeadStatus()
        head_pos.pan  = self.pos_pan
        head_pos.tilt = self.pos_tilt
        self.head_pub.publish(head_pos)

    def head_track_marker(self):
        # doing PID control
        if self.marker_x != -1 :
            error_x = (self.frame_width/2) - self.marker_x
            error_x *= 77.32 / self.frame_width
            error_x = (error_x * math.pi)/ 180
            error_x_diff = error_x - self.last_error_x

            P_pan  = self.last_error_x * self.KP_pan
            self.sum_err_pan += error_x * self.dt
            I_pan = self.sum_err_pan * self.KI_pan
            deriv_err_pan = error_x_diff / self.dt
            D_pan = deriv_err_pan * self.KD_pan
            self.last_error_x = error_x
            self.pos_pan += (P_pan + I_pan + D_pan)

            error_y = (self.frame_height/2) - self.marker_y
            error_y *= -1
            error_y *= 61.93 / self.frame_height
            error_y = (error_y * math.pi) /180
            # error_y_diff = error_y - last_error_y

            P_tilt  = self.last_error_y * self.KP_tilt
            self.sum_err_tilt += error_y * self.dt
            I_tilt = self.sum_err_tilt * self.KI_tilt
            deriv_err_tilt = self.sum_err_tilt / self.dt
            D_tilt = deriv_err_tilt * self.KD_tilt	
            self.last_error_y = error_y
            self.pos_tilt += (P_tilt + I_tilt + D_tilt)
            
            head_pos = HeadStatus()
            self.head_move(self.pos_pan, self.pos_tilt)

    def head_limit(self, pos_pan, pos_tilt):
        if pos_pan <= self.pan_min :		
            pos_pan = self.pan_min
        elif pos_pan >= self.pan_max :		
            pos_pan = self.pan_max
        if pos_tilt <= self.tilt_min :		
            pos_tilt = self.tilt_min
        elif pos_tilt >= self.tilt_max :	
            pos_tilt = self.tilt_max

        return pos_pan, pos_tilt

    def spin_body_to_face_to_marker(self):
        if self.marker_lost(40) :
            error_body_angle = 0
        else:
            error_body_angle = self.pos_pan 
            
        if self.state == 'forward' :            
            self.vel_angle = error_body_angle * 0.4
        else :
            self.vel_angle = error_body_angle * 0.5
            
        # limit
        if self.vel_angle > self.max_vel_angle:
            self.vel_angle = self.max_vel_angle
        elif self.vel_angle < self.max_vel_angle * -1:
            self.vel_angle = self.max_vel_angle * -1
        return self.vel_angle

    # def stay_in_line(self):
    #     if self.marker_lost(20):
    #        self.error_body_y = 0

    #     self.vel_y = self.error_body_y * 0.5
    #     # limit
    #     if self.vel_y   > self.max_vel_y:
    #         self.vel_y  = self.max_vel_y
    #     elif self.vel_y < -self.max_vel_y:
    #         self.vel_y  = -self.max_vel_y
    #     return np.round(self.vel_y, 3)
    #     # return 0.0

    def search_marker(self):
        self.pos_pan = 0
        self.pos_tilt += self.tilt_step
        
        if self.pos_tilt >= self.tilt_max or self.pos_tilt <= self.tilt_min:
            self.tilt_step *= -1
        self.pos_pan, self.pos_tilt =  self.head_limit(self.pos_pan, self.pos_tilt)
        self.head_move(self.pos_pan, self.pos_tilt)

    def run(self):
        time.sleep(4)
        while not rospy.is_shutdown():

            if self.state == 'initial' :

                self.motion_state_pub.publish("stand")
                if self.previous_button[1] == 0 and self.button[1] == 1 :
                    self.state = 'forward'
                elif self.previous_button[0] == 0 and self.button[0] == 1 :
                    self.state = 'wait'

                if self.marker_lost(40):
                    self.search_marker()
                else:
                    self.head_track_marker()  

            elif self.state == 'wait':
                self.motion_state_pub.publish('sit')
                self.head_move(0.0, -1.6)
                if self.previous_button[0] == 0 and self.button[0] == 1 :
                    self.state = 'initial'
                    self.xcomp_pub.publish(-0.004)
                    self.bwd_xcomp = True


            elif self.state == 'forward':
                if self.marker_lost(40):
                    self.search_marker()
                    self.walk(0.01, 0.0,0.0)
                else:
                    self.head_track_marker()
                    self.walk(0.01, 0.0, 0.0 + self.spin_body_to_face_to_marker())
                if self.marker_size > self.max_marker_size :
                    self.state = 'backward'
                    
                if self.previous_button[0] == 0 and self.button[0] == 1 :
                    self.state = 'wait'

            elif self.state == 'backward' :
                if self.bwd_xcomp: 
                    bwd_xcomp = -0.017
                    self.xcomp_pub.publish(bwd_xcomp)
                    self.bwd_xcomp = False

                if self.marker_lost(40):
                    self.search_marker()
                    self.walk(-0.04,  0.0, self.vel_angle_offset)

                else:
                    self.head_track_marker()
                    self.walk(-0.04, 0.0,self.vel_angle_offset + self.spin_body_to_face_to_marker())

                if self.previous_button[0] == 0 and self.button[0] == 1 :
                    self.state = 'wait'

            # change the previous button          
            if self.previous_button[0] != self.button[0]:
                if self.button[0] ==  1:
                    self.previous_button[0] = 1
                else:
                    self.previous_button[0] = 0

            if self.previous_button[1] != self.button[1]:
                if self.button[1]:
                    self.previous_button[1] = 1
                else:
                    self.previous_button[1] = 0
            
            # sleep for a while
            self.rate.sleep()

if __name__ == "__main__":
    player = SprintPlayer()
    player.run()
    rospy.loginfo('sprint player is terminated ! ')
