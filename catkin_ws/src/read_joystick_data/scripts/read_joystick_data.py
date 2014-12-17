#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2014, Viktor Nilsson.
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

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDrive

## axes[4] right trigger
## axes[0] left stick left/right
## buttons[1] B?
def callback(data, pub):
    rospy.loginfo(rospy.get_name() + "I heard this: \n buttons[1]==%f, axes[0]==%f, axes[4]==%f", data.buttons[1], data.axes[0], data.axes[4]);
  
    if data.buttons[1] == 0: direction = 1
    else: direction = -1
    degree = joyAngleToDegree(data.axes[0])
    current = speedToCurrent(data.axes[4])*direction
    
    rospy.loginfo("Setting degree to %f\n", degree)
    rospy.loginfo("Setting current to %f\n", current)

    # Create AckermannDrive msg
    drive = AckermannDrive()
    drive.steering_angle = degree
    drive.speed = current
    
    # Publish under topic 'motor_controller_commands'
    pub.publish(drive)
 
    
def init():
    # Publish under topic 'motor_controller_commands'
    pub = rospy.Publisher('motor_controller_commands', AckermannDrive, queue_size=1)

    # Subscribs to 'joy'
    rospy.Subscriber("joy", Joy, callback, pub)


# Converts a keypress (between 1.0 and -1.0) to a current in Ampere.
def speedToCurrent(speed):
  # Sometimes joy_node says that axis[4] is 0 (even though it should be 1.0
  # when not pressed)
  if speed == 0: return 0;

  # For convenience, we want a value between 0.0 and 1.0
  speed = -((speed-1)/2)
  MAX_CURRENT = 6 # Max speed in current

  if speed < 0.2:
    current = 0
  else:
    current = speed*MAX_CURRENT

  return current
 
 
# Converts from joystick angle to steering angle
def joyAngleToDegree(angle):
    degree = 59*angle
    return degree

        
if __name__ == '__main__':
    rospy.init_node('joy_listener_talker', anonymous=False)
    init()
    rospy.spin()

