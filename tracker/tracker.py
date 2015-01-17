# Paints waypoints on a track.
#
# Usage:
# Click left mouse to create waypoint
# Click on same point to remove
# Press 'C' to clear track
# Press SPACE to print track
# Press 'S' to save image of track
# To insert point(s) in between:
#   Hold shift
#   Click starting point (an existing waypoint)
#   Start adding waypoints (still holding shift)
#   Release shift to go back to normal

import rospy, roslib
from estimate_position.msg import Position

import pygame, sys
from pygame.locals import *
from time import gmtime, strftime
from ast import literal_eval

kilroys = []

def main():
  ros_init()

  pygame.init()
  fpsClock = pygame.time.Clock()
  screen = pygame.display.set_mode((434,700))
  pygame.display.set_caption('Tracker')
  mapbg = pygame.image.load('map_700.jpg')
  font = pygame.font.SysFont("monospace", 15)
  black = pygame.Color(0,0,0)
  label = font.render("x=0, y=0", 1, (0,0,0))
  id_text = font.render("-", 1, (0,0,0))
  track,g_track = read_track()
  shift_mode = False
  prev_pressed = None

  while True:
    screen.blit(mapbg, [0,0])

    # Draw waypoints with alternating colors
    color = pygame.Color(0,255,0)
    for point in g_track:
      color = change_color(color)
      pygame.draw.circle(screen, color, (point), 6)

    if pygame.key.get_mods() & pygame.KMOD_SHIFT:
      shift_mode = True
    else:
      shift_mode = False
      prev_pressed = None

    # Event loop 
    for event in pygame.event.get():
      if event.type == pygame.QUIT:
        pygame.quit()
        sys.exit()
      if event.type == MOUSEMOTION:
        m_x,m_y = event.pos
        x,y = scale_point(m_x,m_y)
        label = font.render("x=%.2f, y=%.2f" % (x,y), 1, (0,0,0))
        # If it is a waypoint here, print the number
        if in_track(g_track,m_x,m_y):
          wp_id = where_in_track(g_track,m_x,m_y)[0]
          where = "wp #%i" % wp_id
          id_text = font.render(where, 1, (0,0,0))
        else:
          id_text = font.render("-", 1, (0,0,0))
      # Shift-mode 
      elif event.type == MOUSEBUTTONDOWN and shift_mode:
        m_x,m_y = event.pos
        if not prev_pressed:
          if in_track(g_track,m_x,m_y):
            prev_pressed = where_in_track(g_track,m_x,m_y)[0]
          else:
            print("You need to select an existing starting point")
        else:
          add_point(prev_pressed+1,m_x,m_y,track,g_track)
          prev_pressed += 1 
      elif event.type == MOUSEBUTTONDOWN:
        m_x,m_y = event.pos
        # If not in track, create new waypoint
        if not in_track(g_track,m_x,m_y):
          track.append((x,y))
          g_track.append((m_x,m_y))
          print("(%.2f,%.2f)" % (x,y))
        # Else remove the existing waypoint
        else:
          (i,x1,y1) = where_in_track(g_track,m_x,m_y)
          print("Removed (%.2f,%.2f)" % (track[i][0],track[i][1]))
          track.pop(i)
          g_track.pop(i)
      # Clear track
      elif event.type == KEYDOWN and event.key == K_c:
        del track[:]
        del g_track[:]
        track_str = ""
        print("Cleared track")
      # Print track
      elif event.type == KEYDOWN and event.key == K_SPACE:
        print("wps = " + track_to_string(track))
      # Save track to image and textfile (.track)
      elif event.type == KEYDOWN and event.key == K_s and track:
        trackname = strftime("track_%Y-%m-%d-%H:%M:%S", gmtime())
        text_file = open(trackname + ".track", "w")
        text_file.write("%s" % track_to_string(track))
        text_file.close()
        pygame.image.save(screen, trackname + ".jpg")
        print("Saved track to " + trackname + ".[jpg/track]")
      # Change state to "add waypoint in between"
      elif event.type == KEYDOWN and event.key == K_r:
        if state != "add_between":
          state = "add_between"
          print("Click on two points then on where to put the new one") 
        else: 
          state = "normal"
          print("Normal mode")

    # Paint lines between all received positions
    #pygame.draw.lines(screen, black, False, kilroys, width=2)

    screen.blit(label, (150,540))
    screen.blit(id_text, (190,560))
    pygame.display.update()
    fpsClock.tick(30)

def read_track():
  if len(sys.argv) == 2:
    g_track = []
    with open (sys.argv[1], "r") as track_file:
      track = literal_eval(track_file.read())
    if not track or type(track[0]) is not tuple or type(track[0][0]) is not float:
      track = []
    else:
      for (x,y) in track:
        g_track.append(rescale_point(x,y))
    return (track, g_track)
  else:
    return ([],[])

def add_point(index,m_x,m_y,track,g_track):
  (x,y) = scale_point(m_x,m_y)
  track.insert(index, (x,y))
  g_track.insert(index, (m_x,m_y))
  print("(%.2f,%.2f)" % (x,y))

def in_track(track,x,y):
  for x1,y1 in track:
    if abs(x-x1) <= 6 and abs(y-y1) <= 6:
      return True
  return False

def where_in_track(track,x,y):
  for i, (x1,y1) in enumerate(track):
    if abs(x-x1) <= 6 and abs(y-y1) <= 6:
      return (i,x1,y1)

def track_to_string(track):
  if not track: 
    return "There is no waypoints to be printed"
  else:
    track_str = "["
    for (x,y) in track:
      track_str += "(%.2f,%.2f)," % (x,y)
    return track_str[:-1] + "]"

def scale_point(x,y):
  x = float(x) / 70
  y = -float(y) / 70
  return (x,y)

def rescale_point(x,y):
  x = int(x * 70)
  y = -int(y * 70)
  return (x,y)

def change_color(color):
  color.r = (color.r+15) % 255
  color.g = (color.g-15) % 255
  return color

def callback_position(pos):
  global kilroys
  kilroys.append((pos.x,pos.y))

def ros_init():
  rospy.init_node("tracker", anonymous=True)
  # Subscribe to Position
  rospy.Subscriber("Position", Position, callback_position)
  rospy.spin()

if __name__ == '__main__':
  main()
