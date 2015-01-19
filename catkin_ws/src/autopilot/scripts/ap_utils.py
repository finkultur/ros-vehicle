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

import math
import rospy # Only for logging/debugging
import config

prev_error = 0
i_term = 0

# Returns the difference between angle to next waypoint and heading of the car.
def get_angle_error(px,py,cx,cy,heading):
  length = math.sqrt(pow((px-cx),2) + pow((py-cy),2))
  cxx = length * math.cos(heading)
  cyy = length * math.sin(heading)
  pxx = px-cx
  pyy = py-cy
  angle = math.atan2(pyy, pxx) - math.atan2(cyy, cxx)

  angle = angle % (2*math.pi)
  if angle > math.pi:
    angle -= 2*math.pi

  #rospy.loginfo("heading: %f, (%f, %f) -> (%f, %f), angle error is %f" % 
  #              (heading, cx, cy, px, py, angle))
  return angle 


# Given input, returns the distance between the circle arc (that the car is
# believed to travel) and the checkpoint p.
def is_on_track(waypoint,cx,cy,heading,steering_angle):
  px,py = waypoint
  R = config.L/(math.tan(steering_angle))
  angle_to_origo = math.pi/2+heading
  o_x = R*math.cos(math.pi-angle_to_origo)
  o_y = R*math.sin(math.pi-angle_to_origo)
  distance_o_to_p = math.sqrt(pow((px-o_x),2)+pow((py-o_y),2))

  return distance_o_to_p-R
  

# Every time we get a new position,
# we adjust the steering angle
# input: wp = (x,y) of next waypoint
#        (cx,cy) = current position of car
#        heading = current heading of car (not steering angle)
# output: a new heading in degrees
def update_steering_angle(wp,cx,cy,heading,steering_angle):
  global i_term, prev_error
  px,py = wp
  angle_error = get_angle_error(px,py,cx,cy,heading)

  p_term = angle_error * config.K_p
  i_term += (angle_error / POS_UPDATE_FREQ) * config.K_i
  d_term = ((angle_error - prev_error) / POS_UPDATE_FREQ) * config.K_d

  steering_angle = p_term + i_term + d_term
  prev_error = angle_error

  return max(config.MIN_STEERING_ANGLE, 
             min(steering_angle, config.MAX_STEERING_ANGLE))


def is_point_behind(point,car_x,car_y,heading):
  x,y = point
  # This is the distance between the rear axis and the imaginary line which the
  # given point is or is not behind. E.g., if equal to 0.3 [m], every point
  # behind the front axis is said to be behind.
  d = 0.1

  # Get point of car front
  (front_x,front_y) = get_new_point(car_x,car_y,d,heading)
  # Calculate a point p1 that is to the left of front
  (p1x,p1y) = get_new_point(front_x,front_y,10,heading+math.pi/2)
  # Calculate a point p2 that is to the right of front
  (p2x,p2y) = get_new_point(front_x,front_y,10,heading-math.pi/2)

  # This catches some corner-cases due to rounding errors
  if heading == math.pi:
    return car_x-d < x 
  elif heading == 0:
    return car_x+d > x 

  if (heading < math.pi):
    return is_point_behind_line(x,y,p1x,p1y,p2x,p2y)
  else:
    return is_point_behind_line(x,y,p2x,p2y,p1x,p1y)
 
 
# Returns a new point from angle and length of vector
def get_new_point(x,y, length, heading):
  new_x = x + length * math.cos(heading)
  new_y = y + length * math.sin(heading)
  return (new_x,new_y)


# Returns true if (x,y) is under the line that is between the points
# (px1,px1) and (px2,py2).
def is_point_behind_line(x,y,px1,py1,px2,py2):
  v1 = (px2-px1, py2-py1)
  v2 = (px2-x, py2-y)
  xp = v1[0]*v2[1] - v1[1]*v2[0]

  if xp > 0:
    return True
  elif xp < 0:
    return False
  else:
    return False

def is_close_enough(wp,x,y,margin):
  wp_x,wp_y = wp
  if abs(wp_x-x) < margin and abs(wp_y-y) < margin:
    return True
  else:
    return False

