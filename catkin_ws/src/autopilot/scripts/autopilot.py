#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2014, Viktor Nilsson and Herman Fransson.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

import rospy, sys, roslib
import ap_utils, config
import math, time
from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDrive
from estimate_position.msg import Position

last_drive_msg = AckermannDrive()
current_steering_angle = 0
current_wp = 0
finished = False

# Saved track to track_2015-01-18-15:06:51.[jpg/track]
wps = [(4.29,-0.60),(4.64,-0.66),(4.99,-0.77),(5.24,-0.94),(5.43,-1.17),(5.57,-1.40),(5.66,-1.66),(5.69,-1.89),(5.70,-2.03),(5.70,-4.83),(5.61,-5.20),(5.41,-5.61),(5.06,-5.94),(4.09,-6.63),(3.64,-6.81),(3.23,-6.89),(2.67,-6.83),(2.27,-6.67),(1.94,-6.39),(1.64,-6.07),(1.43,-5.66),(1.40,-5.09),(1.40,-2.94),(1.53,-2.41),(1.83,-1.96),(2.17,-1.70),(2.63,-1.56),(3.44,-1.53),(3.97,-1.70),(4.24,-1.91),(4.53,-2.31),(4.64,-2.80),(4.63,-3.33),(4.46,-3.81),(4.19,-4.21),(4.07,-4.50),(3.97,-4.93),(3.94,-5.39),(4.03,-5.80),(4.19,-6.11),(4.39,-6.34),(4.70,-6.67),(5.17,-7.29),(5.33,-7.77),(5.33,-8.23),(5.20,-8.61),(4.93,-8.99),(4.47,-9.21),(3.94,-9.29),(2.16,-9.30),(1.81,-9.27),(1.37,-9.06),(1.03,-8.70),(0.79,-8.14),(0.69,-7.21),(0.66,-6.47),(0.66,-2.00),(0.79,-1.57),(1.04,-1.16),(1.37,-0.91),(1.63,-0.77),(1.84,-0.71),(2.10,-0.69)]


# This is run every time a new Position is received.
def callback_position(pos, pub):
  global last_drive_msg, current_steering_angle, current_wp, wps, finished

  #rospy.loginfo(rospy.get_name() + 
  #  "Got a new position: x=%f, y=%f, heading=%f\n" %
  #  (pos.x, pos.y, pos.heading))

  pos.heading = pos.heading % (2*math.pi)
  drive = AckermannDrive()

  # If we are close enough to current waypoint, aim for next.
  if is_close_enough(wps[current_wp], pos.x, pos.y, config.WP_MARGIN):
    if current_wp < len(wps)-1:
      rospy.loginfo("I reached waypoint %i!" % (current_wp))
      current_wp += 1
    elif current_wp == len(wps)-1:
      rospy.loginfo("I reached my destination!\n")
      finished = True
  #elif ap_utils.is_point_behind(wps[current_wp],
  #                              pos.x, pos.y, pos.heading) and not finished:
  #  rospy.loginfo("I missed waypoint %i!" % (current_wp))
  #  current_wp += 1

  if not finished:
    drive.speed = config.SPEED
  else:
    drive.speed = 0.00
  
  # Set speed to 0 if you are outside your "comfort zone"
  if not (config.MIN_X < x < config.MAX_X and config.MIN_Y < y < MAX_Y):
    drive.speed = 0.0

  # Set steering angle based on current position and next waypoint
  current_steering_angle = ap_utils.update_steering_angle(
    wps[current_wp], pos.x, pos.y, pos.heading, current_steering_angle)
  drive.steering_angle = math.degrees(current_steering_angle)
 
  # Only publish messages if it differs from the last one
  if drive != last_drive_msg:
    last_drive_msg = drive
    pub.publish(drive)


def autopilot():
  # Sleep before starting. If parameter not set, default to 5 seconds.
  time.sleep(int(rospy.get_param('Autopilot/wait_time', 5)))

  rospy.init_node("autopilot", anonymous=True)
  pub = rospy.Publisher('mc_cmds', AckermannDrive, queue_size=10)
  rospy.Subscriber("Position", Position, callback_position, pub)
  rospy.spin()


if __name__ == '__main__':
  try:
    autopilot()
  except rospy.ROSInterruptException: pass

