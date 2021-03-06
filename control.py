#!/usr/bin/env python

import sys
import rospy
import thread
import threading
import time
import mavros
import std_msgs.msg
import math
import sensor_msgs.msg

import signal, os

from math import *
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode ,thrust
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped 
from mavros.utils import *
from mavros import setpoint as SP
from tf.transformations import quaternion_from_euler
from quaternion import Quaternion

import signal, os
import sys

############flight_state##################
#"Interrupt"
#"OBSTACLE_AVOID"
#"OBSTACLE_CLEAR"
#"SET_POINT"
##########################################

current_state = False

attitude_pos_pub = None
attitude_pos_msg = None
rate = None
throttle_pub = None
throttle_msg = None
last_pos_z = 0
flight_state = "None"
local_pos_pub = None
set_mode_client = None
q = Quaternion()

flag_front = 0
flag_right = 0
flag_left  = 0 

def state_cb(msg):
    current_state = msg

int_count = 0
to_land = False

offset  = 0.55 #0.66
land_thrust = 0.52
target_pos_z = 1.0
max_thrust = 0.61
min_thrust = 0.5
p = 0.01
i = 0.0025
i_force = 0

def INT_handler(signum, frame):
    global int_count
    if int_count > 0:
        global set_mode_client
    
        print 'Signal handler called with signal', signum
        resp = set_mode_client.call(0, 'AUTO.LAND')
        print ("SetMode AUTO.LAND  state = %r" % resp)    
        sys.exit()
    else:
        global to_land
        global throttle_msg
        throttle_msg.data = land_thrust
        print "to land"
        to_land = True
    int_count = int_count + 1
        #resp = set_mode_client.call(0, 'AUTO.LOITER')
        #print ("SetMode AUTO.LOITER  state = %r" % resp) 

################for global/re_alt ##################
##################################################


def set_attitude():
    global attitude_pos_pub
    global attitude_pos_msg
    global rate
    global throttle_pub
    global throttle_msg
    
    #throttle_msg.data = throttle
    #print(s)
    throttle_pub.publish(throttle_msg)
    attitude_pos_pub.publish(attitude_pos_msg)
    rate.sleep()


def laser_cb(msg):
    pos_z = msg.data/1000.0
    global last_pos_z
    global throttle_msg
    global i_force
    s_force = p * (target_pos_z-pos_z)
    d_force =  -1*(pos_z - last_pos_z)*math.sqrt(p)
    last_pos_z = pos_z
    i_force = i_force + i * (target_pos_z-pos_z)
    if to_land == True :
        throttle_msg.data = land_thrust
    elif (s_force + d_force + i_force + offset) > max_thrust:
        throttle_msg.data = max_thrust
        print "thrust reach max"
    elif (s_force + d_force + i_force + offset) < min_thrust:
        throttle_msg.data = min_thrust
        print "thrust reach min"
    else :
        throttle_msg.data = s_force + d_force + i_force + offset   



thresthold = 500
####left 001
####right 010
####front 100

def laser_front_cb(msg):
    global flag_front
    if msg.data < thresthold:
	    flag_front = 1
    else:
        flag_front = 0

def laser_right_cb(msg):
    global flag_right
    if msg.data < thresthold:
        flag_right = 1
    else:
        flag_right = 0

def laser_left_cb(msg):
    global flag_left
    if msg.data < thresthold:
        flag_left = 1
    else:
        flag_left = 0

def set_pos_msg(msg,x,y,z):
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z

def set_attitude_msg(attitude_pos_msg,q):
    attitude_pos_msg.pose.orientation.x = q.x
    attitude_pos_msg.pose.orientation.y = q.y
    attitude_pos_msg.pose.orientation.z = q.z
    attitude_pos_msg.pose.orientation.w = q.w

def set_thrust(req):
    global offset
    offset = req.thrust
    return True

###########mission function################
def mission(msg):
    global flight_state
    global local_pos_pub
    if flight_state == "SET_POINT":
        set_pos_msg(msg,0,0,2)
        local_pos_pub.publish(msg)
        rate.sleep()
    elif flight_state == "ALT_CTL":
        
        set_attitude()
    elif flight_state == "OBSTACLE_AVOID":
        set_attitude(1,0.533,"",0.99144,-0.13053,0,0)
    elif flight_state == "OBSTACLE_CLEAR":
        set_attitude(1,0.516,"",1,0,0,0)
        flight_state = "SET_POINT"    


def main():
    global attitude_pos_pub
    global attitude_pos_msg
    global rate
    global throttle_pub
    global throttle_msg
    global flight_state
    global local_pos_pub
    global set_mode_client
    global q
    rospy.init_node('rosmav_test_client', anonymous=True)
    mavros.set_namespace()  # initialize mavros module with default namespace
    #state_sub = rospy.Subscriber('mavros/state', State, state_cb, queue_size=10)
    throttle_msg = std_msgs.msg.Float64();
    local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    
    throttle_pub = rospy.Publisher('/mavros/setpoint_attitude/att_throttle', std_msgs.msg.Float64, queue_size=10)

    attitude_pos_pub = rospy.Publisher('/mavros/setpoint_attitude/attitude', PoseStamped, queue_size=10)
    
#data_sub = rospy.Subscriber('/mavros/global_position/rel_alt', std_msgs.msg.Float64, data_cb ,queue_size=1)

    distance_sub = rospy.Subscriber('/mavros/ultrasonic/down',std_msgs.msg.Int16, laser_cb, queue_size=1)

    distance_left_sub = rospy.Subscriber('/mavros/ultrasonic/left',std_msgs.msg.Int16, laser_left_cb, queue_size=1)

    distance_right_sub = rospy.Subscriber('/mavros/ultrasonic/right',std_msgs.msg.Int16, laser_right_cb, queue_size=1)

    distance_front_sub = rospy.Subscriber('/mavros/ultrasonic/front',std_msgs.msg.Int16, laser_front_cb, queue_size=1)


    thrust_srv = rospy.Service('/mavros/set_thrust', thrust, set_thrust)

    rate = rospy.Rate(10)   # 10hz

    while (current_state and current_state.connected):
        rate.sleep()

    msg = SP.PoseStamped(
            header=SP.Header(
            frame_id="base_footprint",  # no matter, plugin don't use TF
                stamp=rospy.Time.now()),    # stamp should update
        )
    

    attitude_pos_msg = SP.PoseStamped(
            header=SP.Header(
            frame_id="base_footprint",  # no matter, plugin don't use TF
                stamp=rospy.Time.now()),    # stamp should update
        )

#    throttle_msg = std_msgs.msg.Float64();
    throttle_msg.data = 0.55

    # Set the signal handler
    signal.signal(signal.SIGINT, INT_handler)

    q_stable = Quaternion()
################### Take off#####

    set_pos_msg(msg,0,0,2)
    
    for i in range(0, 20, 1):
        local_pos_pub.publish(msg)
        rate.sleep()

    print "Takeoff !!!"

    resp = set_mode_client.call(0, 'OFFBOARD')
    print ("SetMode state = %r" % resp)

    resp = arming_client.call(True)
    print ("Arming state = %r" % resp)



#for x in range(0, 50):
#    	local_pos_pub.publish(msg)
#        rate.sleep()
    flight_state = "ALT_CTL"
#########Loop################
    print "takeoff"
    set_attitude_msg(attitude_pos_msg,q)

    for x in range(0, 30):
        throttle_pub.publish(std_msgs.msg.Float64(0.54))
        attitude_pos_pub.publish(attitude_pos_msg)
        rate.sleep() 
        
    #set_pos_msg(msg,0,0,4)
    #for x in range(0, 50):
    #    local_pos_pub.publish(msg)
    #    rate.sleep()
    print "mission"
    while not rospy.is_shutdown():
        mission(msg)
        if flight_state=="Interrupt":
            print "Interrupt by User"
            break
        if to_land == True:
            set_attitude_msg(attitude_pos_msg,q_stable)
        else:
            q.set_q(q.q_stable)
            if flag_front == 1:
                print "backward"
                q.backward()
            if flag_right == 1:
                print "leftward"
                q.shift_left()
            if flag_left == 1:
                print "rightward"
                q.shift_right()
            set_attitude_msg(attitude_pos_msg,q)

####### Landing #######
    print("landing")
    set_pos_msg(msg,0,0,5)
    msg.pose.position.x = 0
    msg.pose.position.y = 0
    msg.pose.position.z = 5


####### need time to land ###################
    resp = set_mode_client.call(0, 'AUTO.LAND')
    print ("SetMode state = %r" % resp)
#    for x in range(0, 100):
#        rate.sleep() 
	
    try:
        pass
    except:
        rospy.loginfo("PX4 Ctl Shutting down")
	
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass



