import socket
import struct

import datetime, keyboard, argparse, traceback
from math import sin,cos,tan,radians,atan,sqrt,degrees,radians
from visualization import *
from time import time

parser = argparse.ArgumentParser()
parser.add_argument("-l", "--log", action='store_true', help='a logging flag')
parser.add_argument("-v", "--verbose", action='store_true', help='a printing flag')
args = parser.parse_args()

# open socket
socket_handler = socket.socket(socket.AF_INET6,socket.SOCK_DGRAM)
socket_handler.bind(('',2019))

if args.log:
    now = datetime.datetime.now()
    filename = str(now.year)+"-"+str(now.month)+"-"+str(now.day)+"-"+str(now.hour)+"-"+str(now.minute)+"-"+str(now.second)+".csv"
    file = open(filename,"w")

# network = Network.initialize(logging=args.log)

def consoleUpdate(event):
    global network
    now = time()
    if event.event_type == 'down':
        print("")
        print(now)
        print(network)
        print("")

keyboard.hook_key('u', lambda event: consoleUpdate(event), suppress=False)

while True:
    try:
        # wait for a request
        message, dist_addr = socket_handler.recvfrom(1024)

        timestamp = time()

        hisAddress     = dist_addr[0]
        hisPort        = dist_addr[1]

        #ASN = struct.unpack('<HHB',message[0:5])
        #addr = format(struct.unpack('<H',message[5:7])[0], 'x')

        pulse_track_count = 5

        built_pulses = []
        last_ind = -12
        for i in range(0, pulse_track_count):
            offset = -i*8
            start = struct.unpack('<I',''.join([chr(b) for b in message[-16+offset:-12+offset]]))[0]
            end = struct.unpack('<I',''.join([chr(b) for b in message[-20+offset:-16+offset]]))[0]
            built_pulses.append((start, end))

            last_ind = -20+offset

        theta = struct.unpack('<f',''.join([chr(b) for b in message[last_ind-4:last_ind]]))[0]
        phi = struct.unpack('<f',''.join([chr(b) for b in message[last_ind-8:last_ind-4]]))[0]
        radial = struct.unpack('<f',''.join([chr(b) for b in message[last_ind-12:last_int-8]]))[0]

        # network.update(raw_pulses=built_pulses)

        data_phi = 'phi: ' + str(phi)
        data_theta = 'theta: ' + str(theta)
        data_radial = 'radial: ' + str(radial)
        #data_addr = 'addr: ' + str(addr)
        #data_asn = 'time[s]: ' + str(0.01*(ASN[0] + ASN[1]*(2**16) + ASN[2]*(2**16)))

        sep = ", "

        data = data_phi + sep + data_theta + sep + data_radial

        if args.verbose:
            print(data)
        if args.log:
            file.write(data + '\n')
    except Exception, err:
        print('Receive failed.')
        print('Printing traceback...')
        traceback.print_exc()
