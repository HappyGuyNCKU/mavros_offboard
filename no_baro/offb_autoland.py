#/usr/bin/env python
import signal
import sys
import rospy
import mavros
import std_msgs.msg
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State,Waypoint,WaypointList,PositionTarget
from mavros_msgs.srv import CommandBool,SetMode,WaypointPull,WaypointPush,thrust
from sensor_msgs.msg import NavSatFix

# callback method for state sub
current_state = State() 
offb_set_mode = SetMode

thottle = 0.1

def state_cb(state):
    global current_state
    current_state = state

current_gps = NavSatFix()
current_gps.status.status = -2 
def gps_cb(gps_value):
    global current_gps
    current_gps = gps_value
def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    set_mode_client(base_mode=0, custom_mode="AUTO.LAND")
    sys.exit(0)

def set_attitude_msg(attitude_pos_msg,w,x,y,z):
    attitude_pos_msg.pose.orientation.x = x
    attitude_pos_msg.pose.orientation.y = y
    attitude_pos_msg.pose.orientation.z = z
    attitude_pos_msg.pose.orientation.w = w


throttle_msg = std_msgs.msg.Float64();
throttle_msg.data = 0.7

def set_thrust(req):
    global throttle_msg
    throttle_msg.data = req.thrust
    return True

mavros.set_namespace()

local_pos_pub = rospy.Publisher(mavros.get_topic('setpoint_position', 'local'), PoseStamped, queue_size=10)
#local_pos_raw_pub = rospy.Publisher(mavros.get_topic('setpoint_row', 'local'), PositionTarget, queue_size=10)
state_sub = rospy.Subscriber(mavros.get_topic('state'), State, state_cb)
##### GPS#####
#gps_sub = rospy.Subscriber(mavros.get_topic('global_position','global'), NavSatFix, gps_cb)

arming_client = rospy.ServiceProxy(mavros.get_topic('cmd', 'arming'), CommandBool)
set_mode_client = rospy.ServiceProxy(mavros.get_topic('set_mode'), SetMode) 

throttle_pub = rospy.Publisher('/mavros/setpoint_attitude/att_throttle', std_msgs.msg.Float64, queue_size=10)
attitude_pos_pub = rospy.Publisher('/mavros/setpoint_attitude/attitude', PoseStamped, queue_size=10)
thrust_srv = rospy.Service('/mavros/set_thrust', thrust, set_thrust)

print "pos1"

a=0
################################################ setpoint_position #######################################
pose = PoseStamped()
pose.pose.position.x = 0
pose.pose.position.y = 0
pose.pose.position.z = 4
pose.pose.orientation.x = 1
pose.pose.orientation.y = 1
pose.pose.orientation.z = 0
pose.pose.orientation.w = 0


attitude_pos_msg = PoseStamped()
set_attitude_msg(attitude_pos_msg,1,0,0,0)

print "pos2"

############################################### setpoint_raw ##############################################
#pose_raw = PositionTarget()
#pos_raw.position.x = 0
#pos_raw.position.y = 0
#pos_raw.position.z = 1
#pos_raw.velocity.x = 50
#pos_raw.velocity.y = 50
#pos_raw.velocity.z = 50


def position_control():
    rospy.init_node('offb_node', anonymous=True)
    print "main"
    saturation = 0
    prev_state = current_state
    prev_gps = current_gps
    rate = rospy.Rate(20.0) # MUST be more then 2Hz
##### GPS  ###    
#while current_gps.status.status < 0 :
#        rate.sleep()
#    gps_sub.unregister()
   
	
    # send a few setpoints before starting
    for i in range(100):
        #attitude_pos_pub.publish(attitude_pos_msg)
        local_pos_pub.publish(pose)
        #local_pos_raw_pub.publish(pose_raw)
        rate.sleep()
   
    signal.signal(signal.SIGINT, signal_handler)
    # wait for FCU connection
    while not current_state.connected:
        rate.sleep()

    #set_mode_client(base_mode=0, custom_mode="AUTO.TAKEOFF")
    resp = set_mode_client(base_mode=0, custom_mode="OFFBOARD")
    print ("SetMode state = %r" % resp)

    #last_request = rospy.get_rostime()
    arming_client(True)
    while not rospy.is_shutdown():
#        now = rospy.get_rostime()
#        
#        if current_state.mode == "STABILIZED" and (now - last_request > rospy.Duration(5.)):
#            a=1
#        elif current_state.mode != "OFFBOARD" and (now - last_request > rospy.Duration(5.)):
#            set_mode_client(base_mode=0, custom_mode="OFFBOARD")
#            last_request = now 
#        else:
#            if not current_state.armed and (now - last_request > rospy.Duration(5.)):
#               arming_client(True)
#               last_request = now 

############# TAKEOFF TEST ######################
        throttle_pub.publish(throttle_msg)
        attitude_pos_pub.publish(attitude_pos_msg)
        #if not current_state.armed:
        #    arming_client(True)



        # older versions of PX4 always return success==True, so better to check Status instead
#        if prev_state.armed != current_state.armed:
#            rospy.loginfo("Vehicle armed: %r" % current_state.armed)
#        if prev_state.mode != current_state.mode: 
#            rospy.loginfo("Current mode: %s" % current_state.mode)
#        prev_state = current_state

        # Update timestamp and publish pose 
#        pose.header.stamp = rospy.Time.now()
#        local_pos_pub.publish(pose)
        #local_pos_raw_pub.publish(pose_raw)
        rate.sleep()

if __name__ == '__main__':
    try:
        position_control()
    except rospy.ROSInterruptException:
        pass
