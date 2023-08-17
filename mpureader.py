import serial
import struct
from math import pi


def get_float(list):
    # print(list)
    aa = bytearray(list)
    return struct.unpack('<f', aa)[0]


state = "HEADER"
header = [11, 37]
header_pos = 0

ser = serial.Serial("/dev/ttyACM0", baudrate=115200)
while (True):
    if state == "HEADER":
        s = ser.read(1)
        b = list(s)[0]
        if b == header[header_pos]:
            header_pos += 1
        else:
            header_pos = 0

        if header_pos >= len(header):
            state = "PAYLOAD"

    else:
        s = ser.read(24)
        bytes = list(s)
        X = get_float(bytes[0:4])
        Y = get_float(bytes[4:8])
        Z = get_float(bytes[8:12])
        Pitch = get_float(bytes[12:16]) / pi * 180
        Yaw = get_float(bytes[16:20]) / pi * 180
        Roll = get_float(bytes[20:24]) / pi * 180
        print(
            f"X:{X:.2f}    Y:{Y:.2f}    Z:{Z:.2f}    P:{Pitch:.2f}    R:{Roll:.2f}    Y:{Yaw:.2f}\n"
        )
        header_pos = 0
        state = "HEADER"
