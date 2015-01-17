# Paints coordinates on a track.
#
# Usage:
# Click left mouse to create waypoint
# Click on same point to remove
# Press 'C' to clear track
# Press SPACE to print track
# Press 'S' to save image of track
# To insert a point between two points:
#   Press 'R'
#   Click on first point
#   Click on second point
#   Click to create waypoint
#   (If no point is added, exit this mode by pressing R again)

import pygame, sys
from pygame.locals import *
from time import gmtime, strftime
from ast import literal_eval

def main():
  pygame.init()
  fpsClock = pygame.time.Clock()
  screen = pygame.display.set_mode((434,700))
  pygame.display.set_caption('Tracker')
  mapbg = pygame.image.load('map_700.jpg')
  font = pygame.font.SysFont("monospace", 15)
  label = font.render("x=0, y=0", 1, (0,0,0))
  id_text = font.render("no waypoint here", 1, (0,0,0))
  track = read_track()
  state = "normal"
  fst_point = None 
  snd_point = None

  while True:
    screen.blit(mapbg, [0,0])

    # Draw waypoints with alternating colors
    color = pygame.Color(0,255,0)
    for point in track:
      color = change_color2(color)
      pygame.draw.circle(screen, color, (point), 6)

    for event in pygame.event.get():
      if event.type == pygame.QUIT:
        pygame.quit()
        sys.exit()
      if event.type == MOUSEMOTION:
        x,y = event.pos
        # If it is a waypoint here, print the number
        if in_track(track,x,y):
          wp_id = track.index(where_in_track(track,x,y))
          where = "wp #%i" % wp_id
          id_text = font.render(where, 1, (0,0,0))
        else:
          id_text = font.render("", 1, (0,0,0))
        (x,y) = scale_point(x,y)
        pos = "x=%.2f, y=%.2f" % (x,y)
        label = font.render(pos, 1, (0,0,0))
      elif event.type == MOUSEBUTTONDOWN:
        x,y = event.pos
        # Add a point in between two other
        if state == "add_between":
          if not fst_point and in_track(track,x,y):
            fst_point = where_in_track(track,x,y)
          elif not snd_point and in_track(track,x,y):
            snd_point = where_in_track(track,x,y)
          elif fst_point and snd_point and fst_point != snd_point:
            track.insert(track.index(snd_point), event.pos)
            (x,y) = scale_point(x,y)
            pos = "(%.2f,-%.2f)" % (x, y)
            print(pos)
            fst_point = None
            snd_point = None
            state = "normal"
          else:
            print("There is no existing point there")
        # If not in track, create new waypoint
        elif not in_track(track,x,y):
          track.append(event.pos)
          (x,y) = scale_point(x,y)
          pos = "(%.2f,-%.2f)" % (x, y)
          print(pos)
        # Else remove the existing waypoint
        else:
          x1,y1 = where_in_track(track,x,y)
          track.remove((x1,y1))
          print("Removed (%.2f,-%.2f)" % scale_point(x1,y1))
      # Clear track
      elif event.type == KEYDOWN and event.key == K_c:
        del track[:]
        track_str = ""
        print("Cleared track")
      # Print track
      elif event.type == KEYDOWN and event.key == K_SPACE:
        print("wps = " + track_to_string(track))
      # Save track to image and textfile (.track)
      elif event.type == KEYDOWN and event.key == K_s:
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

    screen.blit(label, (150,110))
    screen.blit(id_text, (190,170))
    pygame.display.update()
    fpsClock.tick(30)

def read_track():
  if len(sys.argv) == 2:
    with open (sys.argv[1], "r") as track_file:
      track = literal_eval(track_file.read())
    print(track[0][0])
    if not track or type(track[0]) is not tuple or type(track[0][0]) is not float:
      track = []
    else:
      for i, (x,y) in enumerate(track):
        track[i] = rescale_point(x,y)
  return track

def in_track(track,x,y):
  for x1,y1 in track:
    if abs(x-x1) <= 6 and abs(y-y1) <= 6:
      return True
  return False

def where_in_track(track,x,y):
  for x1,y1 in track:
    if abs(x-x1) <= 6 and abs(y-y1) <= 6:
      return (x1,y1)

def track_to_string(track):
  if not track: 
    return "There is no waypoints to be printed"
  else:
    track_str = "["
    for (x,y) in track:
      (x,y) = scale_point(x,y)
      track_str += "(%.2f,-%.2f)," % (x,y)
    return track_str[:-1] + "]"

def scale_point(x,y):
  x = float(x) / 0.7 / 100
  y = float(y) / 0.7 / 100
  return (x,y)

def rescale_point(x,y):
  x = int(x * 70)
  y = -int(y * 70)
  return (x,y)

def change_color(color):
  r,g,b = color
  r = (r+15) % 255
  g = (g-15) % 255
  return (r,g,b)

def change_color2(color):
  color.r = (color.r+15) % 255
  color.g = (color.g-15) % 255
  return color
 
if __name__ == '__main__':
  main()
