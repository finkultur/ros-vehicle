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

## Reads data from the STM32F3-Discover board and publishes that data to the
## 'imu_data' topic. Code based on various ROS-tutorials.

import rospy, serial
from std_msgs.msg import String
from read_imu_data.msg import IMUData

def talker():
    ser = serial.Serial('/dev/ttyACM0')

    pub = rospy.Publisher('imu_data', IMUData, queue_size=10)
    rospy.init_node('imu_talker', anonymous=True)
    r = rospy.Rate(1) # 1Hz
    while not rospy.is_shutdown():
      ser.write('r')
      ser.flush()
      #str = "Time is %s\n"%rospy.get_time()

      data = ser.readline()
      msg = parseIMUData(data) 

      rospy.loginfo(msg)
      pub.publish(msg)
      r.sleep()


# Parses data from the IMU that is on the form
# "float,float,float:float,float,float:float,float,float".
# Creates and returns a IMUData-message
def parseIMUData(indata):
  data = indata.split(':')
  gyro = data[0].split(',')
  acc = data[1].split(',')
  mag = data[2].split(',')

  msg = IMUData()
  msg.gyro0 = float(gyro[0])
  msg.gyro1 = float(gyro[1])
  msg.gyro2 = float(gyro[2])
  msg.acc0 = float(acc[0])
  msg.acc1 = float(acc[1])
  msg.acc2 = float(acc[2])
  msg.mag0 = float(mag[0])
  msg.mag1 = float(mag[1])
  msg.mag2 = float(mag[2])
  return msg
   
        
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
