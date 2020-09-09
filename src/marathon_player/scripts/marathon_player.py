#! /usr/bin/env python
import math
import time
import rospy
import roslib
import numpy as np
from geometry_msgs.msg import Pose2D, Twist
from std_msgs.msg import String, Int16, Int32MultiArray
from robot_msgs.msg import HeadStatus

class MarathonPlayer():
    def __init__(self):
        self.back_count = 0
        # self.pixel_count = 0
        self.spin_time = 4.8
        # left = 0 ; forward = 1 ; right = 2
        # self.marker_list = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        # self.marker_list_index = 0
        
        # position of the line center
        self.line_center_x = -1
        self.line_center_y = -1
        
        self.turn_count = 0

        # state of body
        self.state = String()
        self.state = "initial"

        # state of velocity
        self.vel_x     = 0
        self.vel_y     = 0
        self.vel_angle = 0

        # limit of vel_angle
        self.max_vel_angle = 0.4

        # position of head
        self.pos_pan  =  0.0
        self.pos_tilt = -0.5

        self.pan_step = 0.005

        # the limit of pan an d tilt
        self.pan_min  = -0.6
        self.pan_max  =  0.6
        self.tilt_min = -1.5
        self.tilt_max = -0.8    

        # camera
        self.frame_height = 240 #rospy.get_param('/frame_height')
        self.frame_width  = 320 #rospy.get_param('/frame_width')

        # button state
        self.button = [0, 0]

        # count line center loss
        self.count_line_center_loss = 0

        # rate
        self.freq = 60
        self.rate = rospy.Rate(self.freq)

        # PID control
        self.sum_err_pan  = 0
        self.sum_err_tilt = 0
        self.last_error_x = 0
        self.last_error_y = 0

        self.KP_pan  = 0.03 #0.04
        self.KI_pan  = 0.0
        self.KD_pan  = 0.0

        self.dt      = 1.0 / float(self.freq)

        # button state
        self.button = [0, 0]
        self.previous_button = [0, 0]

        # publisher
        self.motion_vel_pub   = rospy.Publisher("/motion/cmd_vel", Twist,      queue_size=1)
        self.motion_state_pub = rospy.Publisher("/motion/state",   String,     queue_size=1)
        self.head_pub         = rospy.Publisher("/head/pos",       HeadStatus, queue_size=1)

        # subscriber
        rospy.Subscriber('/get_line_center',    Pose2D,             self.line_center_callback)
        rospy.Subscriber("/button/state",       Int32MultiArray,    self.button_callback)
        # rospy.Subscriber('/get_pixel_count',    Int16,              self.pixel_count_callback)

    # callback functions of subscribers
    def line_center_callback(self, pos_msg):
        self.line_center_x = pos_msg.x
        self.line_center_y = pos_msg.y

    def button_callback(self, pos_msg):
        self.button[0] = pos_msg.data[0]
        self.button[1] = pos_msg.data[1]

    # def pixel_count_callback(self, count_msg):
    #     self.pixel_count = count_msg.data

    # control the movement of head
    def head_move(self, pan, tilt):
        self.pos_pan, self.pos_tilt =  self.head_limit(pan, tilt)
        head_pos = HeadStatus()
        head_pos.pan  = self.pos_pan
        head_pos.tilt = self.pos_tilt
        self.head_pub.publish(head_pos)

    # prevent head from overtilting
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

    def head_track_line_center(self):
        if self.line_center_x != -1 :
            error_x = (self.frame_width/2) - self.line_center_x
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
            
            self.head_move(self.pos_pan, self.pos_tilt)
    def go_back_to_line(self, threshold):
        if self.self.line_center_x != -1 :
            self.back_count += 1
            if self.back_count >= threshold:
                return True
        else:
            return False
    # def ready_to_turn(self, threshold):
    #     if self.line_center_x != -1 :
    #         self.turn_count += 1
    #         if self.turn_count >= threshold :
    #             return True
    #     else:
    #         self.turn_count = 0
    #         return False

    def line_center_lost(self, threshold):
        if self.line_center_x == -1:		
            self.count_line_center_loss += 1
            if self.count_line_center_loss >= threshold :
                return True
        else :
            self.count_line_center_loss = 0
            return False

    # find the center of the line
    def search_line_center(self):
        self.pos_pan += self.pan_step
        
        if self.pos_pan >= self.pan_max or self.pos_pan <= self.pan_min:
            self.pan_step *= -1
        self.head_move(self.pos_pan, self.pos_tilt)
    
    # make robot move
    def walk(self, x, y, angle):
        self.vel_x = x
        self.vel_y = y
        self.vel_angle = angle

        velocity = Twist()
        velocity.linear.x = x
        velocity.linear.y = y
        velocity.linear.z = angle
        self.motion_vel_pub.publish(velocity)

    # face to marker
    def spin_body_to_face_to_line_center(self):
        if self.line_center_lost(20) :
            error_body_angle = 0
        else:
            error_body_angle = self.pos_pan 
                       
        self.vel_angle = error_body_angle * 0.25

        if self.vel_angle > self.max_vel_angle:
            self.vel_angle = self.max_vel_angle
        elif self.vel_angle < -self.max_vel_angle:
            self.vel_angle = -self.max_vel_angle

        return self.vel_angle

    def run(self):

        time.sleep(4)

        while not rospy.is_shutdown():
            if self.state == 'initial' :
                self.motion_state_pub.publish("stand")
                if self.line_center_lost(20):
                    self.search_line_center()
                else:
                    self.head_track_line_center()  

                # while button pressed
                if self.previous_button[1] == 0 and self.button[1] == 1 :
                    self.state = 'forward'
                elif self.previous_button[0] == 0 and self.button[0] == 1 :
                    self.state = 'wait'

            elif self.state == 'wait':
                self.motion_state_pub.publish('sit')
                self.head_move(0.0, -0.5)

                # while button pressed               
                if self.previous_button[0] == 0 and self.button[0] == 1 :
                    self.state = 'initial'
                if self.previous_button[1] == 0 and self.button[1] == 1 :
                    self.marker_list_index -= 1

            elif self.state == 'forward' :
                # print("forward")
                if self.line_center_lost(40):
                    self.search_line_center()
                    self.walk(0.0, 0.0, 0.0)
                else:
                    self.head_track_line_center()
                    self.walk(0.03, 0.0,self.spin_body_to_face_to_line_center())


                    # if self.marker_list[self.marker_list_index] == 0:
                    #     self.state = 'left'
                    # elif self.marker_list[self.marker_list_index] == 1:
                    #     self.state = 'searching'
                    # elif self.marker_list[self.marker_list_index] == 2:
                    #     self.state = 'right'
                    # self.marker_list_index += 1
                # print(self.marker_list_index)
                # while buuton pressed
                if self.previous_button[0] == 0 and self.button[0] == 1 :
                    self.state = 'wait'

            elif self.state == 'searching':
                self.head_move(0.0, -0.5)
                self.walk(0.03, 0.0, 0.0)
                if self.line_center_lost(30):
                    self.state = 'forward'
                # print("Search")
                if self.previous_button[0] == 0 and self.button[0] == 1 :
                    self.state = 'wait'

            elif self.state == 'right':
                self.head_move(0.0, -0.5)
                # print("spining right")
                self.walk(0.0, 0.0,-0.1)
                time.sleep(self.spin_time)
                self.state = 'searching'
                
            
            elif self.state == 'left':
                # print("spining left")
                self.head_move(0.0, -0.5)
                self.walk(0.0, 0.0,0.1)
                time.sleep(self.spin_time)
                self.state = 'searching'

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
            self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('marathon_player')
    rospy.loginfo('marathon_player activated ! ')

    player = MarathonPlayer()
    player.run()

    rospy.loginfo('marathon_player is terminated ! ')