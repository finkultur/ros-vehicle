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
"""wps = [(0.0,0.0),(0.5,0.0),(1.0,0.0),(1.5,0.0),(2.0,0.0),
       (2.5,0.5),(3.0,1.0),(3.0,1.5),(3.0,2.0),(2.5,1.5),
       (2.0,2.0),(1.5,2.0),(1.0,2.0),(0.5,2.0),(0.0,2.0)
      ]
"""
wps = [(0,0),(1,0),(2,1),(2,2),(1,2.5),(0,2),(0,0)]

def callback_position(pos, pub):
  global current_steering_angle, current_wp, wps

  #rospy.loginfo(rospy.get_name() + 
  #  "Got a new position: x=%f, y=%f, heading=%f\n" %
  #  (pos.x, pos.y, pos.heading))

  drive = AckermannDrive()

  # If we are close enough to current waypoint, aim for next.
  # If at final waypoint, stop the car.
  if (abs(wps[current_wp][0]-pos.x) < 0.3) and 
     (abs(wps[current_wp][1]-pos.y) < 0.3):
    if current_wp < len(wps)-1:
      rospy.loginfo("I reached waypoint %i!" % (current_wp))
      current_wp += 1
    elif curreent_wp == len(wps)-1:
      rospy.loginfo("I reached my destination!\n")
      drive.speed = 0.0
    else:
      # STOP THE CAR
      drive.speed = 0.0
      pass
  else:
    drive.sped = 0.2
  
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

