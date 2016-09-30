#!/usr/bin/env python

import rospy
import mavros
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State 
from mavros_msgs.srv import CommandBool, SetMode
from sensor_msgs.msg import NavSatFix

# callback method for state sub
current_state = State() 
offb_set_mode = SetMode
def state_cb(state):
    global current_state
    current_state = state

# callback method for gps pos sub
current_gps = NavSatFix()
def gps_pos_cb(nav):
    global current_gps
    current_gps = nav
    
    
mavros.set_namespace('/mavros')
local_pos_pub = rospy.Publisher(mavros.get_topic('setpoint_position', 'local'), PoseStamped, queue_size=10)
#gps_pos_pub = rospy.Subscriber(mavros.get_topic('global_position','global'), NatSatFix, queue_size=10)
state_sub = rospy.Subscriber(mavros.get_topic('state'), State, state_cb)
gps_pos_sub = rospy.Subscriber(mavros.get_topic('global_position', 'global'), NavSatFix, gps_pos_cb)
arming_client = rospy.ServiceProxy(mavros.get_topic('cmd', 'arming'), CommandBool)
set_mode_client = rospy.ServiceProxy(mavros.get_topic('set_mode'), SetMode) 

pose = PoseStamped()
pose.pose.position.x = 0
pose.pose.position.y = 0
pose.pose.position.z = 2

#gps_pose = NavSatFix()
#gps_pose.altitude = 10
#gps_pose.latitude = current_gps.latitude
#gps_pose.longitude = current_gps.longitude


def position_control():
    rospy.init_node('offb_node', anonymous=True)
    prev_state = current_state
    rate = rospy.Rate(20.0) # MUST be more then 2Hz

    # send a few setpoints before starting
    for i in range(100):
        local_pos_pub.publish(pose)
        rate.sleep()
    
    # wait for FCU connection
    while not current_state.connected:
        rate.sleep()

    print (current_gps)
    last_request = rospy.get_rostime()
    count = 0
    while not rospy.is_shutdown():
        now = rospy.get_rostime()
        if current_state.mode != "OFFBOARD" and (now - last_request > rospy.Duration(5.)):
            set_mode_client(base_mode=0, custom_mode="OFFBOARD")
            print "1"
            last_request = now 
        else:
            if not current_state.armed and (now - last_request > rospy.Duration(5.)):
               arming_client(True)
               print "2"
               last_request = now 

        # older versions of PX4 always return success==True, so better to check Status instead
        if prev_state.armed != current_state.armed:
            rospy.loginfo("Vehicle armed: %r" % current_state.armed)
        if prev_state.mode != current_state.mode: 
            rospy.loginfo("Current mode: %s" % current_state.mode)
        prev_state = current_state

        # Update timestamp and publish pose 
        pose.header.stamp = rospy.Time.now()
        local_pos_pub.publish(pose)
        #print "gps latitude:".format(current_gps.latitude)
        #print "gps longitude: ".format(current_gps.longitude)
        rate.sleep()
        count = count +1
        if count%100 == 0:
            print count
        if count > 500:
            break
	
    set_mode_client(base_mode=0, custom_mode="AUTO.LAND")

if __name__ == '__main__':
    try:
        position_control()
    except rospy.ROSInterruptException:
        pass
