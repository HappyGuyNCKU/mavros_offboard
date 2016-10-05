#!/usr/bin/env python

import sys
import rospy
import thread
import threading
import time
import mavros

from math import *
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped 
from mavros.utils import *
from mavros import setpoint as SP
from tf.transformations import quaternion_from_euler


current_state = False

def state_cb(msg):
    current_state = msg


if __name__ == "__main__":
    rospy.init_node('rosmav_test_client', anonymous=True)
    mavros.set_namespace()  # initialize mavros module with default namespace
    state_sub = rospy.Subscriber('mavros/state', State, state_cb, queue_size=10)
    local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
    arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
    set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)  # nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode")
    
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

    for i in range(0, 100, 1):
        local_pos_pub.publish(msg)
        rate.sleep()

    resp = set_mode_client.call(0, 'OFFBOARD')
    print ("SetMode state = %r" % resp)

    resp = arming_client.call(True)
    print ("Arming state = %r" % resp)

    print ("msg = %r" % msg)

    try:
        while(True):
            local_pos_pub.publish(msg)
            rate.sleep()
    except rospy.ROSInterruptException:
        try:
            arming.call(False)
        except:
            rospy.loginfo("PX4 Ctl Shutting down")

    #pub = SP.get_pub_position_local(queue_size=10)
    
    #pub = rospy.Publisher(mavros.get_topic('setpoint_position', 'local'), PoseStamped, queue_size=10)
    #arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
    #arming.call(True)

    #rate = rospy.Rate(10)   # 10hz

    #msg = SP.PoseStamped(
    #    header=SP.Header(
    #    frame_id="base_footprint",  # no matter, plugin don't use TF
    #    stamp=rospy.Time.now()),    # stamp should update
    #)

    #try:
    #    while not rospy.is_shutdown():
    #        msg.pose.position.x = 0.0
    #        msg.pose.position.y = 0.0
    #        msg.pose.position.z = 3.0

            # For demo purposes we will lock yaw/heading to north.
    #        yaw_degrees = 0  # North
    #        yaw = radians(yaw_degrees)
    #        quaternion = quaternion_from_euler(0, 0, yaw)
    #        msg.pose.orientation = SP.Quaternion(*quaternion)

    #        pub.publish(msg)
    #        rate.sleep()
    #except rospy.ROSInterruptException:
    #    try:
    #        arming.call(False)
    #    except:
    #        rospy.loginfo("PX4 Ctl Shutting down")

    #rospy.loginfo("Climb")
    #setpoint.set(0.0, 0.0, 3.0, 0)
    #setpoint.set(0.0, 0.0, 10.0, 5)

    #rospy.loginfo("Sink")
    #setpoint.set(0.0, 0.0, 8.0, 5)

    #rospy.loginfo("Fly to the right")
    #setpoint.set(10.0, 4.0, 8.0, 5)

    #rospy.loginfo("Fly to the left")
    #setpoint.set(0.0, 0.0, 8.0, 5)    
