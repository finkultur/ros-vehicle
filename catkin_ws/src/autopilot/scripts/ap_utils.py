import math
import rospy

L = 0.3
K_p = 0.9
MIN_STEERING_ANGLE = math.radians(-22.0)
MAX_STEERING_ANGLE = math.radians(22.0)

# Returns the difference between angle to next checkpoint
# and heading of the car
def get_angle_error(px,py,cx,cy,heading):
  # Get length of vector
  length = math.sqrt(pow((px-cx),2) + pow((py-cy),2))
  cxx = length * math.cos(heading)
  cyy = length * math.sin(heading)
  pxx = px-cx
  pyy = py-cy
  angle = math.atan2(pyy, pxx) - math.atan2(cyy, cxx)

  rospy.loginfo("heading: %f, (%f, %f) -> (%f, %f), angle error is %f" % 
                (heading, cx, cy, px, py, angle))
  
  if angle > math.pi:
    angle -= 2*math.pi 

  return angle 


# Given input, returns the distance between the circle arc (that the car is
# believed to travel) and the checkpoint p.
def is_on_track(px,py,cx,cy,heading,steering_angle):
  global L
  #heading = math.radians(heading)
  #steering_angle = math.radians(steering_angle)
  R = L/(math.tan(steering_angle))
  angle_to_origo = math.pi/2+heading
  o_x = R*math.cos(math.pi-angle_to_origo)
  o_y = R*math.sin(math.pi-angle_to_origo)
  distance_o_to_p = math.sqrt(pow((px-o_x),2)+pow((py-o_y),2))

  return distance_o_to_p-R
  

# Every time we get a new position,
# we adjust the steering angle
# input: (px,py) = next checkpoint
#        (cx,cy) = current position of car
#        heading = current heading of car (not steering angle)
# output: a new heading in degrees
def update_steering_angle(px,py,cx,cy,heading,steering_angle):
  angle_error = get_angle_error(px,py,cx,cy,heading)

  # If angle error is less than something, set s_a to 0 
  #if abs(angle_error) < math.radians(5):
  #  return 0.0

  if abs(steering_angle) > 1:
    distance_error = is_on_track(px,py,cx,cy,heading,steering_angle)
    # If is_on_track returns a positive number,
    # we need to decrease the steering_angle (and vice versa)
    if abs(distance_error) < 0.1:
      # Do nothing
      pass
    """
    elif distance_error > 0:
      # decrease steering_angle
      steering_angle *= 0.9
    elif (distance_error < 0):
      # increase steering angle
      steering_angle *= 1.1
    """
  steering_angle = angle_error * K_p 

  return max(MIN_STEERING_ANGLE, min(steering_angle, MAX_STEERING_ANGLE))


def is_point_behind_front(x,y,car_x,car_y,heading):
  global L

  # Get point of car front
  (front_x,front_y) = get_new_point(car_x,car_y,0.1,heading)
  # Calculate a point p1 that is to the left of front
  (p1x,p1y) = get_new_point(front_x,front_y,10,heading+math.pi/2)
  # Calculate a point p2 that is to the right of front
  (p2x,p2y) = get_new_point(front_x,front_y,10,heading-math.pi/2)

  # This catches some corner-cases due to rounding errors
  if heading == math.pi:
    return car_x-L < x 
  elif heading == 0:
    return car_x+L > x 

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


# This is needed since all image viewers has (0,0) in top left corner
# [(1,2),(3,4)] => [(1,-2),(3,-4)]
def negate_y(array):
  for i, (x,y) in enumerate(array):
     array[i] = (x,-y)
  return array

