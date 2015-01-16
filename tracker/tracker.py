# Paints coordinates on the given track.
# Click to create waypoint
# Click on same point to remove
# Press 'C' to clear
# Press SPACE to print
# Press 'S' to save image of track

import pygame, sys
from pygame.locals import *

def main():
  pygame.init()

  fpsClock = pygame.time.Clock()
  screen = pygame.display.set_mode((434,700))
  pygame.display.set_caption('Tracker')
  mapbg = pygame.image.load('map_700.jpg')

  redColor = pygame.Color(255,0,0)
  greenColor = pygame.Color(0,255,0)
  blueColor = pygame.Color(0,0,255)
  whiteColor = pygame.Color(255,255,255)

  font = pygame.font.SysFont("monospace", 15)
  label = font.render("x=0, y=0", 1, (0,0,0))
  mouseX, mouseY = 0,0
  
  track = []
  track_id = 0

  while True:
    screen.blit(mapbg, [0,0])

    for point in track:
      pygame.draw.circle(screen, greenColor, (point), 6)

    for event in pygame.event.get():
      if event.type == pygame.QUIT:
        pygame.quit()
        sys.exit()
      if event.type == MOUSEMOTION:
        x,y = event.pos
        (x,y) = scale_point(x,y)
        pos = "x=%.2f, y=%.2f" % (x,y)
        label = font.render(pos, 1, (0,0,0))
      elif event.type == MOUSEBUTTONDOWN:
        x,y = event.pos
        if not_in_track(track,x,y):
          track.append(event.pos)
          (x,y) = scale_point(x,y)
          pos = "(%.2f,-%.2f)" % (x, y)
          print(pos)
      elif event.type == KEYDOWN and event.key == K_c:
        del track[:]
        track_str = ""
        print("Cleared track")
      elif event.type == KEYDOWN and event.key == K_SPACE:
        print_track(track)
      elif event.type == KEYDOWN and event.key == K_s:
        filename = "track%i.jpg" % track_id
        track_id += 1
        pygame.image.save(screen, filename)
        print("Saved image of track to " + filename)

    screen.blit(label, (150,110))
    pygame.display.update()
    fpsClock.tick(30)

def not_in_track(track,x,y):
  for x1,y1 in track:
    if abs(x-x1) <= 6 and abs(y-y1) <= 6:
      # Remove point
      track.remove((x1,y1))
      (x1,y1) = scale_point(x1,y1)
      print("Removed (%.2f,%.2f) from track" % (x1,y1))
      return False
  return True

def scale_point(x,y):
  x = float(x) / 0.7 / 100
  y = float(y) / 0.7 / 100
  return (x,y)  

def print_track(track):
  if not track: 
    print "There is no waypoints to be printed"
  else:
    track_str = "wps = ["
    for (x,y) in track:
      (x,y) = scale_point(x,y)
      track_str += "(%.2f,-%.2f)," % (x,y)
    print(track_str[:-1] + "]") 
  
 
if __name__ == '__main__':
  main()
