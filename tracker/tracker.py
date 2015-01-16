# Paints coordinates on the given track.
# List of coordinates is returned on exit.

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
  track_str = "wps = ["

  while True:
    screen.fill(whiteColor)
    screen.blit(mapbg, [0,0])

    for event in pygame.event.get():
      if event.type == pygame.QUIT:
        pygame.quit()
        sys.exit()
      if event.type == MOUSEMOTION:
        mouseX, mouseY = event.pos
        pos = "x=" + str(float(mouseX)/100) + ", y=" + str(float(mouseY)/100)
        label = font.render(pos, 1, (0,0,0))
      elif event.type == MOUSEBUTTONDOWN:
        mouseX, mouseY = event.pos
        track.append(event.pos)

        mouseX = float(mouseX) / 0.7 
        mouseY = float(mouseY) / 0.7
        newpos = "(%.2f,-%.2f)," % (mouseX/100, mouseY/100)
        track_str += newpos
        print(newpos[:-1])
      elif event.type == KEYDOWN and event.key == K_c:
        del track[:]
        track_str = "wps = ["
        print("Cleared track")

      if event.type == KEYDOWN and event.key == K_SPACE:
        track_str = track_str[:-1] + "]"
        print(track_str)

    for point in track:
      pygame.draw.circle(screen, greenColor, (point), 6)

    screen.blit(label, (150,110))
    pygame.display.update()
    fpsClock.tick(30)
 
if __name__ == '__main__':
  main()
