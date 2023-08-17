import serial
import struct
from math import pi
import time
import numpy as np
import signal

last_accel = np.array([0.0, 0.0, 0.0])
samples = []
out = open("acc.csv", 'w')


def get_float(list):
    # print(list)
    aa = bytearray(list)
    return struct.unpack('<f', aa)[0]


def read_vals(ser):
    s = ser.read(24)
    bytes = list(s)
    X = get_float(bytes[0:4])
    Y = get_float(bytes[4:8])
    Z = get_float(bytes[8:12])
    Pitch = get_float(bytes[12:16]) / pi * 180
    Yaw = get_float(bytes[16:20]) / pi * 180
    Roll = get_float(bytes[20:24]) / pi * 180
    return X, Y, Z, Pitch, Roll, Yaw

def sig_handler(signum, frame):
    for sample in samples:
        t, X, Y, Z, Pitch, Roll, Yaw = sample
        out.write(f"{t};{X};{Y};{Z};{Pitch};{Yaw};{Roll}\n")
    out.close()
    exit()

state = "HEADER"
header = [11, 37]
header_pos = 0

g_const = 9.8

ser = serial.Serial("/dev/ttyACM0", baudrate=115200)

begin = time.time()
last = time.time()
print_counter = 0

signal.signal(signal.SIGINT, sig_handler)

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
        X, Y, Z, Pitch, Roll, Yaw = read_vals(ser)
        header_pos = 0
        state = "HEADER"
        current = time.time()
        dt = current - last
        last = current
        last_accel = np.array([X, Y, Z]) * g_const
        samples.append([current-begin, X, Y, Z, Pitch, Roll, Yaw])
        print(
            f"X:{X:.2f}    Y:{Y:.2f}    Z:{Z:.2f}    P:{Pitch:.2f}    R:{Roll:.2f}    Y:{Yaw:.2f}\n"
        )

    # print_counter = (print_counter + 1) % 100
    # if(print_counter == 99):
