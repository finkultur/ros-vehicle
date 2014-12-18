import serial

ser = serial.Serial('/dev/ttyUSB0')

cmd = bytearray([0x5A, 0x01, 0x00, 0x00])
ser.write(cmd)
ser.flush()
res = ser.read()
print("Firmware version: " + hex(res) + '\n')

# ???
cmd = bytearray([0x55, 0xE8+1, 0x00, 0x01])
ser.write(cmd)
ser.flush()
res = ser.read()
print("ret: " + hex(res) + '\n')

# Starts the SRF08 at address 0xE8 to ranging
cmd = bytearray([0x55, 0xE8+1, 0x00, 0x01, 0x51])
ser.write(cmd)
ser.flush()
res = ser.read()
print("range succ: " + hex(res) + '\n')

# Read range from SRF08
cmd = bytearray([0x55, 0xE8+1, 0x00, 0xFF])
while True:
  cmd[0] = 0x55
  cmd[1] = 0xE8+1
  cmd[2] = 0x00
  cmd[3] = 0x04
  ser.write(cmd) # I hope this only writes 4 bytes
  ser.flush()
  cmd[0] = serial.readChar() 
  cmd[1] = serial.readChar()
  cmd[2] = serial.readChar()
  cmd[3] = serial.readChar()
  if cmd[3] != 0xFF:
    break

range_ = cmd[2]<<8|cmd[3]
print("ret: " + range_ + '\n')

#
# Insert catch-system-error here
#

