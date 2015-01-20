#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2014, Viktor Nilsson.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
#
# tracker.py: Paints waypoints on a track.
#
# Usage:
# Click left mouse to create waypoint
# Click on same point to remove
# Press 'C' to clear track
# Press 'P' to clear received Positions (if plotting enabled)
# Press SPACE to print track to console
# Press 'S' to save image of track
# To insert point(s) in between:
#   Hold shift
#   Click starting point (an existing waypoint)
#   Start adding waypoints (still holding shift)
#   Release shift to go back to normal mode

import rospy, roslib
from estimate_position.msg import Position

import pygame, sys, threading, argparse
from pygame.locals import *
from time import gmtime, strftime
from ast import literal_eval

kilroys = []

def main():
  global lock,kilroys

  # Parse command line arguments
  parser = argparse.ArgumentParser()
  parser.add_argument("--plot", help="Plots position on track",
                      action="store_true")
  parser.add_argument('--track', nargs=1, help="Choose an existing track")
  args = parser.parse_args()

  lock = threading.Lock()
  # Only need to init ROS if plotting of position is enabled
  if args.plot:
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
  track,g_track = read_track(args.track[0])
  shift_mode = False
  prev_pressed = None

  while True:
    screen.blit(mapbg, [0,0])

    # Draw waypoints with alternating colors
    color = pygame.Color(0,255,0)
    for point in g_track:
      color = change_color(color)
      pygame.draw.circle(screen, color, point, 6)
    # Draw received positions
    if args.plot and len(kilroys) > 1:
      for point in kilroys:
        screen.set_at(point, black)

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
          add_point(len(track),m_x,m_y,track,g_track)
        # Else remove the existing waypoint
        else:
          remove_point(m_x,m_y,track,g_track)
      # Clear track
      elif event.type == KEYDOWN and event.key == K_c:
        del track[:]
        del g_track[:]
        track_str = ""
        print("Cleared waypoints")
      # Clear received positions
      elif event.type == KEYDOWN and event.key == K_p:
        lock.acquire()
        try:
          del kilroys[:]
        finally:
          lock.release()
        print("Cleared received positions")
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
        print("# Saved track to " + trackname + ".[jpg/track]")

    screen.blit(label, (150,540))
    screen.blit(id_text, (190,560))
    pygame.display.update()
    fpsClock.tick(30)


def read_track(trackfile):
  g_track = []
  with open (trackfile, "r") as track_file:
    track = literal_eval(track_file.read())
  if not track or type(track[0]) is not tuple or type(track[0][0]) is not float:
    track = []
  else:
    for (x,y) in track:
      g_track.append(rescale_point(x,y))
  return (track, g_track)

def add_point(index,m_x,m_y,track,g_track):
  (x,y) = scale_point(m_x,m_y)
  track.insert(index, (x,y))
  g_track.insert(index, (m_x,m_y))
  print("(%.2f,%.2f)" % (x,y))

def remove_point(m_x,m_y,track,g_track):
  (i,x1,y1) = where_in_track(g_track,m_x,m_y)
  print("Removed (%.2f,%.2f)" % (track[i][0],track[i][1]))
  track.pop(i)
  g_track.pop(i)

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
  color.r = (color.r+10) % 255
  color.g = (color.g-15) % 255
  return color

def callback_position(pos):
  global kilroys, lock
  print("Got a position! x=%f, y=%f, heading=%f" % (pos.x,pos.y,pos.heading))
  x,y = rescale_point(pos.x,pos.y)
  lock.acquire()
  try:
    kilroys.append((x,y))
  finally:
    lock.release()

def ros_init():
  rospy.init_node("tracker", anonymous=True)
  # Subscribe to Position
  rospy.Subscriber("Position", Position, callback_position)

if __name__ == '__main__':
  main()

