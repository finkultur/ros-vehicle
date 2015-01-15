# Paints coordinates on the given track.
# List of coordinates is returned on exit.

import pygame, sys
from pygame.locals import *

def main():
  pygame.init()

  screen = pygame.display.set_mode((434,700))
  pygame.display.set_caption('Tracker')
  mapbg = pygame.image.load('map_700.jpg')

  redColor = pygame.Color(255,0,0)
  greenColor = pygame.Color(0,255,0)
  blueColor = pygame.Color(0,0,255)
  whiteColor = pygame.Color(255,255,255)

  mouseX, mouseY = 0,0
  
  track = []
  track_str = "wps = ["

  while True:
    screen.fill(whiteColor)
    screen.blit(mapbg, [0,0])

    for event in pygame.event.get():
      if event.type == pygame.QUIT:
        track_str = track_str[:-1] + "]"
        print(track_str)
        pygame.quit()
        sys.exit()
      if event.type == MOUSEMOTION:
        mouseX, mouseY = event.pos
      elif event.type == MOUSEBUTTONDOWN:
        mouseX, mouseY = event.pos
        track.append(event.pos)

        mouseX = float(mouseX) / 0.7 
        mouseY = float(mouseY) / 0.7
        newpos = "("+str(float(mouseX)/100) +",-"+ str(float(mouseY)/100)+"),"
        track_str += newpos
        print(newpos)

    for point in track:
      pygame.draw.circle(screen, greenColor, (point), 6)

    pygame.display.update()
 
if __name__ == '__main__':
  main()
