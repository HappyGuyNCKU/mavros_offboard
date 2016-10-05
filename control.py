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
last_pos_z = 1.5
flight_state = "None"
local_pos_pub = None
set_mode_client = None

def state_cb(msg):
    current_state = msg

int_count = 0
to_land = False

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
        throttle_msg.data = 0.605
        print "to land"
        to_land = True
    int_count = int_count + 1
        #resp = set_mode_client.call(0, 'AUTO.LOITER')
        #print ("SetMode AUTO.LOITER  state = %r" % resp) 

################for global/re_alt ##################
def data_cb(data_msg):
    pos_z = data_msg.data
    global last_pos_z
    global throttle_msg
    target_pos_z = 7.6
    max_thrust = 0.85
    p = 0.015
    s_force = p * (target_pos_z-pos_z)
    d_force =  0#-1*(pos_z - last_pos_z)*10*math.sqrt(p)
    last_pos_z = pos_z
    if (s_force + d_force +offset) > max_thrust:
        throttle_msg.data = offset
    else :
        throttle_msg.data = s_force + d_force + offset
    #if pos_z <  1.65 :   #Low
    #    acce_state = 1
    #elif pos_z > 1.45 :    #High
    #    acce_state = -1
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

offset  = 0.64 #0.66
land_thrust = 0.625
target_pos_z = 1.0
max_thrust = 0.7
min_thrust = 0.57
p = 0.03
i = 0.001
i_force = 0

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
    rospy.init_node('rosmav_test_client', anonymous=True)
    mavros.set_namespace()  # initialize mavros module with default namespace
    #state_sub = rospy.Subscriber('mavros/state', State, state_cb, queue_size=10)
    local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    
    throttle_pub = rospy.Publisher('/mavros/setpoint_attitude/att_throttle', std_msgs.msg.Float64, queue_size=10)

    attitude_pos_pub = rospy.Publisher('/mavros/setpoint_attitude/attitude', PoseStamped, queue_size=10)
    
#data_sub = rospy.Subscriber('/mavros/global_position/rel_alt', std_msgs.msg.Float64, data_cb ,queue_size=1)

    distance_sub = rospy.Subscriber('/mavros/down/ultrasonic',std_msgs.msg.Int16, laser_cb, queue_size=1)

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

    throttle_msg = std_msgs.msg.Float64();
    throttle_msg.data = 0.64

    # Set the signal handler
    signal.signal(signal.SIGINT, INT_handler)

    q = Quaternion()
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

    for x in range(0, 50):
        throttle_pub.publish(std_msgs.msg.Float64(0.6))
        attitude_pos_pub.publish(attitude_pos_msg)
        rate.sleep() 
        
    #set_pos_msg(msg,0,0,4)
    #for x in range(0, 50):
    #    local_pos_pub.publish(msg)
    #    rate.sleep()
    print "mission"
    attctl_count = 0
    while not rospy.is_shutdown():
        mission(msg)
        if flight_state=="Interrupt":
            print "Interrupt by User"
            break
        attctl_count = attctl_count +1
        mod = attctl_count % 250
        if mod == 30:
            q.shift_left()
            print "left"
        elif mod == 40:
            q.shift_right()
            print "back"
        elif mod == 70:
            q.shift_right()
            print "right"
        elif mod == 80:
            q.shift_left()
            print "back"
        elif mod == 130:
            q.forward()
            print "forward"
        elif mod == 140:
            q.backward()
            print "back"
        elif mod == 170:
            q.backward()
            print "backward"
        elif mod == 180:
            q.forward()
            print "back"
        elif mod == 230:
            q.rotate_cw()
            print "rotate_cw"
        if to_land == True:
            set_attitude_msg(attitude_pos_msg,q_stable)
        else:
            set_attitude_msg(attitude_pos_msg,q_stable)

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



