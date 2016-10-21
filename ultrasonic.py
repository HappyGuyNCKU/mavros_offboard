#!/usr/bin/python

import RPi.GPIO as GPIO                    #Import GPIO library
import time 							   #Import time library

import rospy
import std_msgs.msg

import sys
import os

def main():
  rospy.init_node('ultrasonic_node', anonymous=True)
  distance_msg = std_msgs.msg.Int16();
  GPIO.setmode(GPIO.BCM)                     #Set GPIO pin numbering 

  GPIO.setwarnings(False)
  if str(sys.argv[1]) == "down":
    TRIG = 15                                  #Associate pin 23 to TRIG
    ECHO = 18                                  #Associate pin 24 to ECHO
    topic = "mavros/ultrasonic/down"
  elif str(sys.argv[1]) == "front":
    TRIG = 20
    ECHO = 21
    topic = "mavros/ultrasonic/front"
  elif str(sys.argv[1]) == "left":
    TRIG = 19
    ECHO = 26
    topic = "mavros/ultrasonic/left"
  elif str(sys.argv[1]) == "right":
    TRIG = 5
    ECHO = 6
    topic = "mavros/ultrasonic/right"
  else:
    print "*********** Warning ***********\nUsing ./ultrasonic.py <down/front/left/right>\nexiting...."
    sys.exit()

  distance_pub = rospy.Publisher(topic,std_msgs.msg.Int16, queue_size=1)
  rate = rospy.Rate(3)

  print "Distance measurement in progress"

  GPIO.setup(TRIG,GPIO.OUT)                  #Set pin as GPIO out
  GPIO.setup(ECHO,GPIO.IN)                   #Set pin as GPIO in


  while not rospy.is_shutdown():
    GPIO.output(TRIG, False)                 #Set TRIG as LOW
    #print "Waitng For Sensor To Settle"

    GPIO.output(TRIG, True)                  #Set TRIG as HIGH
    time.sleep(0.00001)                      #Delay of 0.00001 seconds
    GPIO.output(TRIG, False)                 #Set TRIG as LOW
#   print "pulse"	
    while GPIO.input(ECHO)==0:               #Check whether the ECHO is LOW
      pulse_start = time.time()              #Saves the last known time of LOW pulse
#    print "inside"
#    for x in range(10000):               #Check whether the ECHO is HIGH
#      pulse_end = time.time()
#      if GPIO.input(ECHO)==0:
#        break
#    print "break"
    while GPIO.input(ECHO)==1:
	  pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start #Get pulse duration to a variable

    distance = pulse_duration * 17150        #Multiply pulse duration by 17150 to get distance
    distance = 10 * round(distance, 1)            #Round to two decimal points

    if distance > 20 and distance < 2000:      #Check whether the distance is within range
      distance_msg.data = distance
    elif distance > 2000:  
      distance_msg.data = 2000
    else:
      distance_msg.data = 0                   #display out of range
    distance_pub.publish(distance_msg)
    rate.sleep()
  
  print "exiting..."
  sys.exit()

if __name__ == "__main__":
  try:
    main()
  except rospy.ROSInterruptException:
    pass
