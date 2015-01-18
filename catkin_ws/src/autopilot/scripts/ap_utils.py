import math
import rospy

L = 0.3
K_p = 1.2
K_i = 0.0
K_d = 0.2
prev_error = 0
i_term = 0
MIN_STEERING_ANGLE = math.radians(-22.0)
MAX_STEERING_ANGLE = math.radians(22.0)
sampling_freq = 200 # We get a new Position every 5 ms

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

  angle = angle % (2*math.pi)
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
  global i_term, prev_error
  angle_error = get_angle_error(px,py,cx,cy,heading)

  # If angle error is less than something, set s_a to 0 
  #if abs(angle_error) < math.radians(5):
  #  return 0.0

  """ TODO: Maybe use this stuff
  if abs(steering_angle) > 1:
    distance_error = is_on_track(px,py,cx,cy,heading,steering_angle)
    # If is_on_track returns a positive number,
    # we need to decrease the steering_angle (and vice versa)
    if abs(distance_error) < 0.1:
      # Do nothing
      pass
    elif distance_error > 0:
      # decrease steering_angle
      steering_angle *= 0.9
    elif (distance_error < 0):
      # increase steering angle
      steering_angle *= 1.1
  """

  p_term = angle_error * K_p
  i_term += (angle_error / sampling_freq) * K_i # Is this right?
  d_term = ((angle_error - prev_error) / sampling_freq) * K_d

  steering_angle = p_term + i_term + d_term
  prev_angle_error = angle_error

  return max(MIN_STEERING_ANGLE, min(steering_angle, MAX_STEERING_ANGLE))


def is_point_behind(x,y,car_x,car_y,heading):
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

