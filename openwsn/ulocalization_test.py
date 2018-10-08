import socket
import struct

import traceback
from math import sin,cos,tan,radians,atan,sqrt,degrees,radians
# from visualization import *
from time import time

# open socket
socket_handler = socket.socket(socket.AF_INET6,socket.SOCK_DGRAM)
socket_handler.bind(('',2069))

# network = Network.initialize(logging=args.log)

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

        # modular_ptr = struct.unpack('<B',message[-33:-32])[0]

        built_pulses = []
        for i in range(0, pulse_track_count):
            offset = -i*8
            end = struct.unpack('<I',message[-28+offset:-24+offset])[0]
            start = struct.unpack('<I',message[-32+offset:-28+offset])[0]
            built_pulses.append((start, end))
        built_pulses = list(reversed(built_pulses))

        z = struct.unpack('<f',message[-16:-12])[0]
        y = struct.unpack('<f',message[-20:-16])[0]
        x = struct.unpack('<f',message[-24:-20])[0]

        # network.update(raw_pulses=built_pulses)

        # ptr_data = 'mod: ' + str(modular_ptr)
        data_x = 'pulses: ' + str(int(x))
        data_y = 'attemped tx\'s: ' + str(int(y))
        data_z = 'successful tx\'s: ' + str(int(z))
        #data_addr = 'addr: ' + str(addr)
        #data_asn = 'time[s]: ' + str(0.01*(ASN[0] + ASN[1]*(2**16) + ASN[2]*(2**16)))

        sep = ", "

        data = data_x + sep + data_y + sep + data_z

        print("")
        print(data)
        print(built_pulses)
        print("")

    except Exception, err:
        print('Receive failed.')
        print('Printing traceback...')
        traceback.print_exc()
