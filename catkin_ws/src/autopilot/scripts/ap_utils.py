
import math

L = 0.3
K_p = 0.5
MIN_STEERING_ANGLE = -22.0
MAX_STEERING_ANGLE = 22.0

# Returns the difference between angle to next checkpoint
# and heading of the car
def get_angle_error(px,py,cx,cy,heading):
  #heading = math.radians(heading)

  # Get length of vector
  l = math.sqrt(pow((px-cx),2) + pow((py-cy),2))
  cxx = l * math.cos(heading)
  cyy = l * math.sin(heading)
  pxx = abs(px-cx)
  pyy = abs(py-cy)
  dot_product = cxx*pxx + cyy*pyy
  angle = math.acos(dot_product/(l*l))

  return angle 

# Given input, returns the distance between the circle arc (that the car is
# believed to travel) and the checkpoint p.
def is_on_track(px,py,cx,cy,heading,steering_angle):
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
  if abs(angle_error) < math.radians(5):
    return 0.0

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

#print(math.degrees(get_angle_error(10,10,0,0,90)))
#print(update_steering_angle(10,10,0,0, 45, 22))


# TODO: Fix this!
def is_point_behind_front(x,y,car_x,car_y,heading):
  # Get point of car front
  (front_x,front_y) = get_new_point(car_x,car_y,heading,0.3)
  # Calculate a point p1 that is to the left of front
  (p1x,p1y) = get_new_point(front_x,front_y,10,heading+math.pi/4)
  # Calculate a point p2 that is to the left of front
  (p2x,p2y) = get_new_point(front_x,front_y,10,heading-math.pi/4)

  #if (p1x < p2x):
  #  return is_point_behind_line(x,y,p1x,p1y,p2x,p2y)
  #else:
  #  return is_point_behind_line(x,y,p2x,p2y,p1x,p1y)
  return is_point_behind_line(x,y,p2x,p2y,p1x,p1y)
  

def get_new_point(x,y,length,heading):
  new_x = x + length * math.cos(heading)
  new_y = y + length * math.sin(heading)
  return (new_x,new_y)


# Returns true if (x,y) is under the line that is between the points
# (px0,px0) and (lx1,ly1).
def is_point_behind_line(x,y,px0,py0,px1,py1):
  v1 = (px1-px0, py1-py0)
  v2 = (px1-x, py1-y)
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


