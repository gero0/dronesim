import serial
import struct
from math import pi
from os import system, name
import curses
from curses import wrapper


def get_float(list):
    aa = bytearray(list)
    return struct.unpack('<f', aa)[0]


def get_int(list):
    aa = bytearray(list)
    return struct.unpack('<I', aa)[0]


def get_byte(list):
    return list


# define our clear function
def clear():
    # for windows
    if name == 'nt':
        _ = system('cls')

    # for mac and linux(here, os.name is 'posix')
    else:
        _ = system('clear')


def main(stdscr):
    state = "HEADER"
    header = [11, 37]
    header_pos = 0
    ser = serial.Serial("/dev/ttyACM0", baudrate=115200)
    stdscr.clear()

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
            s = ser.read(62)
            bytes = list(s)
            # print(bytes)
            # exit()
            Pitch = get_float(bytes[0:4]) / 3.14 * 180
            Yaw = get_float(bytes[4:8]) / 6.28 * 360
            Roll = get_float(bytes[8:12]) / 3.14 * 180
            X = get_float(bytes[12:16])
            Y = get_float(bytes[16:20])
            Z = get_float(bytes[20:24])
            press_alt = get_float(bytes[24:28])
            radar_alt = get_float(bytes[28:32])

            stdscr.addstr(0, 0, f"X:{X:.2f} Y:{Y:.2f} Z:{Z:.2f}")

            stdscr.addstr(1, 0, f"P:{Pitch:.2f} R:{Roll:.2f} Y:{Yaw:.2f}")
            stdscr.addstr(2, 0,
                          f"Alt.(P):{press_alt:.2f} Alt (R):{radar_alt:.2f}")

            stdscr.refresh()

            header_pos = 0
            state = "HEADER"


wrapper(main)
