import serial
import struct
import crcmod
import csv
from math import pi
from os import system, name
from curses import wrapper


frame_start = b'\xaa'

def get_float(list):
    aa = bytearray(list)
    return struct.unpack('<f', aa)[0]

def get_int(list):
    aa = bytearray(list)
    return struct.unpack('<I', aa)[0]

# define our clear function
def clear():
    # for windows
    if name == 'nt':
        _ = system('cls')

    # for mac and linux(here, os.name is 'posix')
    else:
        _ = system('clear')


def calculate_modbus_crc(data):
    # Create a CRC-16 Modbus object
    crc16 = crcmod.predefined.Crc('modbus')

    # Update CRC with provided data
    crc16.update(data)

    # Get the calculated CRC value
    crc_value = crc16.crcValue

    # Modbus CRC is little-endian, so swap bytes
    crc_bytes = crc_value.to_bytes(2, byteorder='little')

    return crc_bytes



def main(stdscr):
    state = "HEADER"
    ser = serial.Serial("/dev/ttyUSB0", baudrate=115200)
    stdscr.clear()

    with open('acc.csv', 'w') as accfile:
        with open('gyro.csv', 'w') as gyrofile:
                accwriter = csv.writer(accfile)
                gyrowriter = csv.writer(gyrofile)
                accwriter.writerow(['X', 'Y', 'Z'])
                gyrowriter.writerow(['PR', 'RR', 'YR'])

                while (True):
                    b = ser.read(1)
                    if b != frame_start:
                        continue
                    frame_size = ser.read(1)[0]
                    payload = ser.read(frame_size)
                    crc = ser.read(2)
                    verify_crc = calculate_modbus_crc(payload)
                    if crc[0] == verify_crc[0] and crc[1] == verify_crc[1]:
                        pass
                    else:
                        continue

                    X = get_float(payload[0:4])
                    Y = get_float(payload[4:8])
                    Z = get_float(payload[8:12])
                    Pitch = get_float(payload[12:16])
                    Yaw = get_float(payload[16:20])
                    Roll = get_float(payload[20:24])
                    timestamp = get_int(payload[24:28])

                    accwriter.writerow([timestamp, X,Y,Z])
                    gyrowriter.writerow([timestamp, Pitch,Roll,Yaw])

                    stdscr.addstr(0, 0, f"time: {timestamp}")

                    stdscr.addstr(1, 0, f"X:{X:.2f} Y:{Y:.2f} Z:{Z:.2f}")

                    stdscr.addstr(2, 0, f"PR:{Pitch:.2f} RR:{Roll:.2f} YR:{Yaw:.2f}")

                    stdscr.refresh()


wrapper(main)
# main(None)
