#!/usr/bin/env python
import rospy, sys, roslib
import ap_utils
import math
import time
from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDrive
from estimate_position.msg import Position_msg

current_steering_angle = 0
current_wp = 0
#wps = [(0.0,0.0),(0.5,0.0),(1.0,0.0),(1.5,0.0),(2.0,0.0),(2.5,0.5),(3.0,1.0),(3.0,1.5),(3.0,2.0),(2.5,1.5),(2.0,2.0),(1.5,2.0),(1.0,2.0),(0.5,2.0),(0.0,2.0)]
#wps = [(0,0),(1,0),(2,1),(2,2),(1,2.5),(0,2),(0,0)]

# An ellipse
#wps = [(2.0,0.0),(2.4,0.1),(2.7,0.2),(3.0,0.3),(3.3,0.4),(3.6,0.6),(3.8,0.9),(3.9,1.2),(3.9,1.5),(3.8,1.7),(3.7,2.0),(3.4,2.1),(3.1,2.2),(2.6,2.2),(2.1,2.2),(1.6,2.2),(1.1,2.1),(0.7,1.9),(0.5,1.6),(0.4,1.3),(0.3,1.0),(0.5,0.7),(0.7,0.5),(0.9,0.3),(1.4,0.1),(1.7,0.0)]
# Another ellipse
wps = [(2.0,0.0),(4.0,0.0),(6.4,1.0),(7.1,2.6),(6.2,4.2),(4.4,4.9),(2.7,4.9),(1.1,4.2),(0.5,2.5),(0.9,0.9)]


# An eight. Probably wont work
#wps = {(2.0,0.0),(2.3,0.1),(2.5,0.2),(2.7,0.4),(2.9,0.6),(2.9,0.9),(2.9,1.2),(2.6,1.5),(2.4,1.6),(2.1,1.8),(1.7,2.0),(1.3,2.3),(0.9,2.5),(0.5,2.7),(0.3,2.9),(0.3,3.4),(0.6,3.8),(1.0,4.0),(1.5,4.1),(1.9,4.1),(2.3,3.9),(2.5,3.5),(2.5,3.0),(2.2,2.6),(1.9,2.3),(1.6,2.1),(1.3,1.6),(0.9,1.2),(0.7,0.8),(0.6,0.4),(1.0,0.1),(1.5,0.0)}

# Banana + loop
#wps = {(2.6,0.0),(3.2,0.0),(3.7,0.1),(4.1,0.1),(4.5,0.1),(5.1,0.2),(5.5,0.3),(5.7,0.6),(6.0,0.9),(6.2,1.2),(6.3,1.6),(6.3,2.1),(6.3,2.6),(6.1,2.9),(5.6,3.2),(5.2,3.5),(5.0,4.0),(4.7,4.6),(3.8,5.1),(2.8,5.2),(2.0,5.0),(1.6,4.3),(2.0,3.8),(2.6,3.4),(3.3,3.1),(3.6,2.6),(3.7,1.9),(3.4,1.1),(2.8,0.8),(2.0,0.9),(1.5,1.4),(1.4,2.0),(1.6,2.5),(2.3,2.8),(3.1,2.8),(4.0,2.7),(4.4,2.1),(4.5,1.5),(4.4,0.9),(4.0,0.6),(3.4,0.4),(3.0,0.2),(2.6,0.2)}


def callback_position(pos, pub):
  global current_steering_angle, current_wp, wps

  #rospy.loginfo(rospy.get_name() + 
  #  "Got a new position: x=%f, y=%f, heading=%f\n" %
  #  (pos.x, pos.y, pos.heading))

  drive = AckermannDrive()

  # If we are close enough to current waypoint, aim for next.
  # If at final waypoint, stop the car.
  if (abs(wps[current_wp][0]-pos.x) < 0.3) and (abs(wps[current_wp][1]-pos.y) < 0.3):
    if current_wp < len(wps)-1:
      rospy.loginfo("I reached waypoint %i!" % (current_wp))
      current_wp += 1
      drive.speed = 0.20
    elif current_wp == len(wps)-1:
      rospy.loginfo("I reached my destination!\n")
      drive.speed = 0.0
  else:
    drive.speed = 0.20
  
  # If more than x meters away from start, stop the car
  if abs(pos.x) > 5 or abs(pos.y) > 5:
    #drive.speed = 0.0
    pass

  # Set steering angle based on next waypoint
  current_steering_angle = ap_utils.update_steering_angle(
    wps[current_wp][0],wps[current_wp][1],
    pos.x, pos.y, pos.heading, current_steering_angle)
  drive.steering_angle = math.degrees(current_steering_angle)

  # Publish under topic 'mc_cmds'
  pub.publish(drive)


def autopilot():
  time.sleep(5)

  rospy.init_node("autopilot", anonymous=True)

  # Publish under topic 'mc_cmds'
  pub = rospy.Publisher('mc_cmds', AckermannDrive, queue_size=10)

  # Subscribe to Position
  rospy.Subscriber("Position", Position_msg, callback_position, pub)

  rospy.spin()


if __name__ == '__main__':
  try:
    autopilot()
  except rospy.ROSInterruptException: pass

