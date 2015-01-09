
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


