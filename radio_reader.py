import serial
import struct
from math import pi
from os import system, name

state = "HEADER"
header = [11, 37]
header_pos = 0


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


ser = serial.Serial("/dev/ttyACM0")
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
        Pitch = get_float(bytes[0:4])
        Yaw = get_float(bytes[4:8])
        Roll = get_float(bytes[8:12])
        X = get_float(bytes[12:16])
        Y = get_float(bytes[16:20])
        Z = get_float(bytes[20:24])
        press_alt = get_float(bytes[24:28])
        radar_alt = get_float(bytes[28:32])
        motor_a = get_byte(bytes[33])
        motor_b = get_byte(bytes[34])
        motor_c = get_byte(bytes[35])
        motor_d = get_byte(bytes[36])
        msg_type = get_int(bytes[36:40])
        resp_type = get_int(bytes[40:44])
        msg_time = get_int(bytes[44:48])
        resp_time = get_int(bytes[48:52])
        inp_time = get_int(bytes[52:56])

        clear()
        print(
            f"X:{X:.2f} Y:{Y:.2f} Z:{Z:.2f}    P:{Pitch:.2f} R:{Roll:.2f} Y:{Yaw:.2f}"
        )
        print(
            f"Alt.(P):{press_alt:.2f} Alt (R):{radar_alt:.2f} M1:{motor_a} M2:{motor_b} M3:{motor_c} M4:{motor_d}"
        )
        print(
            f"Last msg type:{msg_type} Last msg time:{msg_time}\n Last resp type:{resp_type} Last resp time:{resp_time}"
        )

        header_pos = 0
        state = "HEADER"
