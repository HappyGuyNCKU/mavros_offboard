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

from math import *
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped 
from mavros.utils import *
from mavros import setpoint as SP
from tf.transformations import quaternion_from_euler


current_state = False

attitude_pos_pub = None
attitude_pos_msg = None
rate = None
throttle_pub = None
throttle_msg = None
last_pos_z = 1.5

def state_cb(msg):
    current_state = msg

#def acce_cb(acce_msg):
#    acce_z = acce_msg.linear_acceleration.z
#    #global acce_state
#    global throttle_msg
#    if acce_z < 9.75:     #accending
#        throttle_msg.data = throttle_msg.data + 0.001
#    elif acce_z > 10.26:    #decending
#        throttle_msg.data = throttle_msg.data - 0.001
#    else:
#        throttle_msg.data = throttle_msg.data + 0.0001*acce_state

def data_cb(data_msg):
    pos_z = data_msg.data
    global last_pos_z
    global throttle_msg
    p = 0.15
    s_force = p * (1.5-pos_z)
    d_force = -1*(pos_z - last_pos_z)*10*math.sqrt(p)
    last_pos_z = pos_z
    if (s_force + d_force +0.513) > 1:
        throttle_msg.data = 0.513
    else :
        throttle_msg.data = s_force + d_force +0.513
    #if pos_z <  1.65 :   #Low
    #    acce_state = 1
    #elif pos_z > 1.45 :    #High
    #    acce_state = -1

def quaternion(c,b,a,d):
    roll=math.atan(2*(a*b+c*d)/(a*a-b*b-c*c+d*d))
    pitch=-1*math.asin(2*(b*d-a*c))
    yaw=math.atan(2*(a*d+b*c)/(a*a+b*b-c*c-d*d))
    print "Quaternion:"
    print "Roll = %f" % roll
    print "Pitch = %f" % pitch
    print "Yaw = %f" % yaw

def set_attitude(dur,throttle,s,w,x,y,z):
    global attitude_pos_pub
    global attitude_pos_msg
    global rate
    global throttle_pub
    global throttle_msg
    
    #throttle_msg.data = throttle
    print(s)
    attitude_pos_msg.pose.orientation.x = x
    attitude_pos_msg.pose.orientation.y = y
    attitude_pos_msg.pose.orientation.z = z
    attitude_pos_msg.pose.orientation.w = w
    for x in range(0,dur):
        throttle_pub.publish(throttle_msg)
        attitude_pos_pub.publish(attitude_pos_msg)
        rate.sleep()

def ultrasonci_cb(msg):
    if msg.data < 30 and msg.data >0:
        print "too close"
        
    

def main():
    global attitude_pos_pub
    global attitude_pos_msg
    global rate
    global throttle_pub
    global throttle_msg
    rospy.init_node('rosmav_test_client', anonymous=True)
    mavros.set_namespace()  # initialize mavros module with default namespace
    #state_sub = rospy.Subscriber('mavros/state', State, state_cb, queue_size=10)
    local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
    arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
    set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)  # nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode")

    attitude_pub = rospy.Publisher('mavros/setpoint_attitude/cmd_vel', TwistStamped, queue_size=10)
    
    throttle_pub = rospy.Publisher('mavros/setpoint_attitude/att_throttle', std_msgs.msg.Float64, queue_size=10)

    attitude_pos_pub = rospy.Publisher('mavros/setpoint_attitude/attitude', PoseStamped, queue_size=10)
    
    data_sub = rospy.Subscriber('mavros/global_position/rel_alt', std_msgs.msg.Float64, data_cb ,queue_size=1)

    #acce_sub = rospy.Subscriber('mavros/imu/data_raw', sensor_msgs.msg.Imu, acce_cb ,queue_size=1)    

    distance_sub = rospy.Subscriber('mavros/ultrasonic',std_msgs.msg.Int16, ultrasonci_cb, queue_size=1)
    rate = rospy.Rate(10)   # 10hz

    while (current_state and current_state.connected):
        rate.sleep()

    msg = SP.PoseStamped(
            header=SP.Header(
            frame_id="base_footprint",  # no matter, plugin don't use TF
                stamp=rospy.Time.now()),    # stamp should update
        )
    msg.pose.position.x = 0
    msg.pose.position.y = 0
    msg.pose.position.z = 2

    attitude_msg = SP.TwistStamped(
            header=SP.Header(
            frame_id="base_footprint",  # no matter, plugin don't use TF
                stamp=rospy.Time.now()),    # stamp should update
        )
    attitude_msg.twist.angular.x = 0.5
    attitude_msg.twist.angular.y = 0
    attitude_msg.twist.angular.z = 0

    attitude_pos_msg = SP.PoseStamped(
            header=SP.Header(
            frame_id="base_footprint",  # no matter, plugin don't use TF
                stamp=rospy.Time.now()),    # stamp should update
        )

    x = attitude_pos_msg.pose.orientation.x = 0.01
    y = attitude_pos_msg.pose.orientation.y = 0.01
    z = attitude_pos_msg.pose.orientation.z = 2
    w = attitude_pos_msg.pose.orientation.w = 0.5

    quaternion(x,y,z,w)
    throttle_msg = std_msgs.msg.Float64();


################### Take off#####
    for i in range(0, 20, 1):
        local_pos_pub.publish(msg)
        rate.sleep()

    resp = set_mode_client.call(0, 'OFFBOARD')
    print ("SetMode state = %r" % resp)

    resp = arming_client.call(True)
    print ("Arming state = %r" % resp)

    #print ("msg = %r" % msg)


    for x in range(0, 0):
    	local_pos_pub.publish(msg)
        rate.sleep()



#########Test################

#############stable###############
    print("pos_msg")
    throttle_msg.data=0.513
    attitude_msg.twist.angular.x = 0
    
    x = attitude_pos_msg.pose.orientation.x = 0
    y = attitude_pos_msg.pose.orientation.y = 0
    z = attitude_pos_msg.pose.orientation.z = 0
    w = attitude_pos_msg.pose.orientation.w = 1
    throttle_pub.publish(throttle_msg)
    for x in range(0, 0):
        attitude_pos_pub.publish(attitude_pos_msg)
        rate.sleep()

##############pos_msg
    throttle_msg.data = 0.513
    #set_attitude(20,0.51,"z axis",0.70711,0,0,0.70711)
    #set_attitude(attitude_pos_pub,attitude_pos_msg,rate,"back",1,0,0,0)
    #set_attitude(15,0.543,"x aixis",0.701057,0.092296,-0.092296,0.701057)
    set_attitude(50,0.516,"back",1,0,0,0)
    set_attitude(30,0.533,"x aixis reverse ",0.99144,-0.13053,0,0)
    set_attitude(80,0.516,"back",1,0,0,0)
############
    set_attitude(30,0.5311,"y axis :pitch",0.99144,0,0.13053,0)
    set_attitude(50,0.513,"back",1,0,0,0)



#############

####### Landing #######
    print("landing")
    msg.pose.position.x = 0
    msg.pose.position.y = 0
    msg.pose.position.z = 1

    for x in range(0, 50):
        local_pos_pub.publish(msg)
        rate.sleep()

    msg.pose.position.x = 0
    msg.pose.position.y = 0
    msg.pose.position.z = 0

    for x in range(0, 10):
        local_pos_pub.publish(msg)
        rate.sleep()   

 
    try:
        print("disarm")
        arming_client.call(False)
    except:
        rospy.loginfo("PX4 Ctl Shutting down")
	
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass



