#!/usr/bin/env python
import rospy, sys, roslib
import ap_utils
import math
import time
from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDrive
from estimate_position.msg import Position

last_drive_msg = AckermannDrive()
current_steering_angle = 0
current_wp = 0
finished = False

#wps = [(0.0,0.0),(0.5,0.0),(1.0,0.0),(1.5,0.0),(2.0,0.0),(2.5,0.5),(3.0,1.0),(3.0,1.5),(3.0,2.0),(2.5,1.5),(2.0,2.0),(1.5,2.0),(1.0,2.0),(0.5,2.0),(0.0,2.0)]
#wps = [(0,0),(1,0),(2,1),(2,2),(1,2.5),(0,2),(0,0)]

# An ellipse
#wps = [(2.0,0.0),(2.4,0.1),(2.7,0.2),(3.0,0.3),(3.3,0.4),(3.6,0.6),(3.8,0.9),(3.9,1.2),(3.9,1.5),(3.8,1.7),(3.7,2.0),(3.4,2.1),(3.1,2.2),(2.6,2.2),(2.1,2.2),(1.6,2.2),(1.1,2.1),(0.7,1.9),(0.5,1.6),(0.4,1.3),(0.3,1.0),(0.5,0.7),(0.7,0.5),(0.9,0.3),(1.4,0.1),(1.7,0.0)]
# Another ellipse
#wps = [(2.0,0.0),(4.0,0.0),(6.4,1.0),(7.1,2.6),(6.2,4.2),(4.4,4.9),(2.7,4.9),(1.1,4.2),(0.5,2.5),(0.9,0.9)]

# An eight. Probably wont work
#wps = [(2.0,0.0),(2.3,0.1),(2.5,0.2),(2.7,0.4),(2.9,0.6),(2.9,0.9),(2.9,1.2),(2.6,1.5),(2.4,1.6),(2.1,1.8),(1.7,2.0),(1.3,2.3),(0.9,2.5),(0.5,2.7),(0.3,2.9),(0.3,3.4),(0.6,3.8),(1.0,4.0),(1.5,4.1),(1.9,4.1),(2.3,3.9),(2.5,3.5),(2.5,3.0),(2.2,2.6),(1.9,2.3),(1.6,2.1),(1.3,1.6),(0.9,1.2),(0.7,0.8),(0.6,0.4),(1.0,0.1),(1.5,0.0)]

# Banana + loop
#wps = {(2.6,0.0),(3.2,0.0),(3.7,0.1),(4.1,0.1),(4.5,0.1),(5.1,0.2),(5.5,0.3),(5.7,0.6),(6.0,0.9),(6.2,1.2),(6.3,1.6),(6.3,2.1),(6.3,2.6),(6.1,2.9),(5.6,3.2),(5.2,3.5),(5.0,4.0),(4.7,4.6),(3.8,5.1),(2.8,5.2),(2.0,5.0),(1.6,4.3),(2.0,3.8),(2.6,3.4),(3.3,3.1),(3.6,2.6),(3.7,1.9),(3.4,1.1),(2.8,0.8),(2.0,0.9),(1.5,1.4),(1.4,2.0),(1.6,2.5),(2.3,2.8),(3.1,2.8),(4.0,2.7),(4.4,2.1),(4.5,1.5),(4.4,0.9),(4.0,0.6),(3.4,0.4),(3.0,0.2),(2.6,0.2)}

#wps = (0,0),(0.46,0.19),(1.48,0.6),(2.22,0.6),(2.86,0.6),(3.66,0.6),(4.4,0.6),(4.83,0.88),(5.26,1.12),(5.5,1.44),(5.55,1.84),(5.53,2.28),(5.53,7.2),(5.4,7.8),(5.2,8.25),(4.7,8.4),(4.3,8.4),(1.8,8.4),(1.1,8.3),(0.7,8.1),(0.7,2.16)

# Forward and turn left
#wps = [(0,0),(3,0),(5,1),(6,2),(6,3.5)]

# Around the whole track (not the inner part)
#wps = [(2.015,0.585),(4.20,0.585),(4.78,0.72),(5.3,1.08),(5.56,1.52),(5.66,1.96),(5.5,5.5),(5.3,7.77),(5.1,8.35),(5,8.7),(4.5,9.0),(3.6,9.3),(3,9.4),(2.3,9.38),(1.3,9.3),(0.9,8.9),(0.6,8.6),(0.5,8.0),(0.46,1.9),(0.74,1.3),(1.12,0.94),(1.6,0.7),(2.015,0.585)]


# Stop in box
#wps = [(2.015, 0.585), (4.28, 1.92), (4.32, 4.20), (4.40, 6.52)]

# Whole outer track
#wps = [(2.015, 0.565),(4.24, 0.56),(5.66,1.96),(5.14,8.16),(2.24,9.3), (0.58, 6.5),(0.58,2.32), (2.015, 0.565)]

# Whole outer track 2 times
wps = [(2.015, 0.565),(4.24, 0.56),(5.66,1.96),(5.14,8.16),(2.24,9.3), (0.58, 6.5),(0.58,2.32), (2.015, 0.565), (4.24, 0.56),(5.66,1.96),(5.14,8.16),(2.24,9.3), (0.58, 6.5),(0.58,2.32), (2.015, 0.565)]

# This is needed since we are getting coordinates that starts with (0,0) in top
# left corner.
wps = ap_utils.negate_y(wps)

def callback_position(pos, pub):
  global last_drive_msg, current_steering_angle, current_wp, wps, finished

  #rospy.loginfo(rospy.get_name() + 
  #  "Got a new position: x=%f, y=%f, heading=%f\n" %
  #  (pos.x, pos.y, pos.heading))

  drive = AckermannDrive()

  # If we are close enough to current waypoint, aim for next.
  # If at final waypoint, stop the car.
  if (abs(wps[current_wp][0]-pos.x) < 0.2) and (abs(wps[current_wp][1]-pos.y) < 0.2):
    if current_wp < len(wps)-1:
      rospy.loginfo("I reached waypoint %i!" % (current_wp))
      current_wp += 1
    elif current_wp == len(wps)-1:
      rospy.loginfo("I reached my destination!\n")
      finished = True
  #elif ap_utils.is_point_behind_front(wps[current_wp][0], wps[current_wp][1],
  #                                  pos.x, pos.y, pos.heading) and not finished:
  #  rospy.loginfo("I missed waypoint %i!" % (current_wp))
  #  current_wp += 1

  if not finished:
    drive.speed = 0.42
  else:
    drive.speed = 0.00
  
  # If more than x meters away from start, stop the car
  if abs(pos.x) > 5 or abs(pos.y) > 5:
    #drive.speed = 0.0
    pass

  # Set steering angle based on next waypoint
  current_steering_angle = ap_utils.update_steering_angle(
    wps[current_wp][0],wps[current_wp][1],
    pos.x, pos.y, pos.heading, current_steering_angle)
  drive.steering_angle = math.degrees(current_steering_angle)

  
  if drive != last_drive_msg:
    last_drive_msg = drive
    # Publish under topic 'mc_cmds'
    pub.publish(drive)


def autopilot():
  # Sleep before starting. If parameter not set, default to 5 seconds.
  time.sleep(int(rospy.get_param('wait_time', '5')))

  rospy.init_node("autopilot", anonymous=True)

  # Publish under topic 'mc_cmds'
  pub = rospy.Publisher('mc_cmds', AckermannDrive, queue_size=10)

  # Subscribe to Position
  rospy.Subscriber("Position", Position, callback_position, pub)

  rospy.spin()


if __name__ == '__main__':
  try:
    autopilot()
  except rospy.ROSInterruptException: pass

