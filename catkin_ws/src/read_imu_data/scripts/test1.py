#!/bin/python

import serial, time

ser = serial.Serial('/dev/ttyACM0')
while True:
  ser.write('r')
  ser.flush()

  data_raw = ser.readline()
  print(data_raw)
  data_raw = ser.readline()
  print(data_raw)
  data_raw = ser.readline()
  print(data_raw)
  time.sleep(1)
  
