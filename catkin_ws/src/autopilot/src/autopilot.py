import rospy
import ap_utils
import math
#from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDrive
#from bldc_mc.msg import MCValues
from estimate_position.msg import Position

current_steering_angle = 0
current_wp = 0
wps = {(0.0,0.0),(0.5,0.0),(1.0,0.0),(1.5,0.0),(2.0,0.0),
       (2.5,0.5),(3.0,1.0),(3.0,1.5),(3.0,2.0),(2.5,1.5),
       (2.0,2.0),(1.5,2.0),(1.0,2.0),(0.5,2.0),(0.0,2.0)
      }


def callback_position(pos, pub):
  rospy.loginfo(rospy.get_name() + 
    "Got a new position: x=%f, y=%f, heading=%f\n" %
    (data.x, data.y, data.heading))

  drive = AckermannDrive()

  # If we are close enough to current waypoint, aim for next.
  # If at final waypoint, stop the car.
  if abs(wps[current_wp][0]-data.x) < 0.2 and
     abs(wps[current[wp][1]-data.y) < 0.2:
    if current_wp < len(wps)-1
      current_wp += 1
    else:
      # STOP THE CAR
      drive.speed = 0.0
      pass
  
  # If more than x meters away from start, stop the car
  if abs(data.x) > 3 or abs(data.y) > 3:
    drive.speed = 0.0

  # Set steering angle based on next waypoint
  current_steering_angle = ap_utils.update_steering_angle(
    wps[current_wp][0],wps[current_wp][1],
    data.x, data.y, heading, current_steering_angle)
  drive.steering_angle = math.degrees(current_steering_angle)

  # Publish under topic 'mc_cmds'
  pub.publish(drive)


def init():
  rospy.init_node("autopilot", anonymous=False)

  # Publish under topic 'mc_cmds'
  pub = rospy.Publisher('mc_cmds', AckermannDrive, queue_size=10)

  # Subscribe to Position
  rospy.Subscriber("Position", Position, callback_position, pub)

  rospy.spin()


if __name__ == '__main__':
  try:
    init()
  except rospy.ROSInterruptException: pass

