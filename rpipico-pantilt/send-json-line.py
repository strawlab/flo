# This program will send a single line of JSON to a serial device.
#
# It serves as a tiny utility to test the parsing of JSON lines
# in the `rpipico-pantilt` firmware.

import sys
import serial

device = sys.argv[1]

ser = serial.Serial(device)
ser.write(b'{"Set":{"pan":1500.0,"tilt":2000.0,"enabled":true}}\n')
ser.close()
