import serial
import struct
import crcmod
import sys
import time
from math import pi

from PyQt6.QtWidgets import QApplication, QMainWindow, QLabel, QWidget, QGridLayout, QLineEdit, QPushButton
from PyQt6.QtCore import  QObject, QThread, pyqtSignal
from dataclasses import dataclass, field


frame_start = b'\xaa'

def to_degrees(a):
    return a / 3.14 * 180

def get_float(list):
    aa = bytearray(list)
    return struct.unpack('<f', aa)[0]

def get_int(list):
    aa = bytearray(list)
    return struct.unpack('<I', aa)[0]

def get_byte(byte):
    return int(byte)

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

@dataclass
class DroneData:
    Pitch: float = 0
    Yaw: float = 0
    Roll: float = 0
    Local_X: float = 0
    Local_Y: float = 0
    Alt_relative: float = 0
    Alt_radar: float = 0
    Alt_absolute: float = 0
    Pitch_sp: float = 0
    Yaw_sp: float = 0
    Roll_sp: float = 0
    Thrust_sp: float = 0
    motors : list = field(default_factory=list) 
    pitch_Kp : float = 0
    pitch_Ki : float = 0
    pitch_Kd : float = 0
    roll_Kp : float = 0
    roll_Ki : float = 0
    roll_Kd : float = 0
    yaw_Kp : float = 0
    yaw_Ki : float = 0
    yaw_Kd : float = 0
    thrust_Kp : float = 0
    thrust_Ki : float = 0
    thrust_Kd : float = 0
    last_response_time : int = 0
    joy0x : float = 0
    joy0y : float = 0
    joy1x : float = 0
    joy1y : float = 0

class DataReader:

    def __init__(self):
        self.ser = serial.Serial("/dev/ttyUSB0", baudrate=115200)
        self.dd = DroneData()

    def write(self, array):
        self.ser.write(array)

    def read_message(self):
        b = self.ser.read(1)
        if b != frame_start:
            return
        msg_type = self.ser.read(1)
        frame_size = self.ser.read(1)[0]
        payload = self.ser.read(frame_size)
        crc = self.ser.read(2)
        verify_crc = calculate_modbus_crc(payload)

        if crc[0] == verify_crc[0] and crc[1] == verify_crc[1]:
            pass
        else:
            print("CRC ERROR")
            return

        self.dd.Pitch = get_float(payload[0:4])
        self.dd.Yaw = get_float(payload[4:8])
        self.dd.Roll = get_float(payload[8:12])
        self.dd.Local_X = get_float(payload[12:16])
        self.dd.Local_Y = get_float(payload[16:20])
        self.dd.Alt_relative = get_float(payload[20:24])
        self.dd.Alt_radar = get_float(payload[24:28])
        self.dd.Alt_absolute = get_float(payload[28:32])
        self.dd.Pitch_sp = get_float(payload[32:36])
        self.dd.Yaw_sp = get_float(payload[36:40])
        self.dd.Roll_sp = get_float(payload[40:44])
        self.dd.Thrust_sp = get_float(payload[44:48])
        self.dd.motors = [get_byte(payload[48]), get_byte(payload[49]), get_byte(payload[50]), get_byte(payload[51])]
        self.dd.pitch_Kp = get_float(payload[52: 56])
        self.dd.pitch_Ki = get_float(payload[56: 60])
        self.dd.pitch_Kd = get_float(payload[60: 64])
        self.dd.roll_Kp = get_float(payload[64: 68])
        self.dd.roll_Ki = get_float(payload[68: 72])
        self.dd.roll_Kd = get_float(payload[72: 76])
        self.dd.yaw_Kp = get_float(payload[76: 80])
        self.dd.yaw_Ki = get_float(payload[80: 84])
        self.dd.yaw_Kd = get_float(payload[84: 88])
        self.dd.thrust_Kp = get_float(payload[88: 92])
        self.dd.thrust_Ki = get_float(payload[92: 96])
        self.dd.thrust_Kd = get_float(payload[96: 100])
        self.dd.last_response_time = get_int(payload[100: 104])
        self.dd.joy0x = get_float(payload[104: 108])
        self.dd.joy0y = get_float(payload[108: 112])
        self.dd.joy1x = get_float(payload[112: 116])
        self.dd.joy1y = get_float(payload[116: 120])

        return self.dd
    


class Worker(QObject):
    progressed= pyqtSignal(DroneData)
    finished= pyqtSignal()
    data_reader = DataReader()

    tx_queue = []


    def run(self):

        while True:
            if len(self.tx_queue) != 0:
                self.data_reader.write(self.tx_queue.pop())
        
            dd= self.data_reader.read_message()
            self.progressed.emit(dd)

        self.finished.emit()
    
    def update_tunings(self, new_dd):
        payload = [
            new_dd.pitch_Kp, new_dd.pitch_Ki, new_dd.pitch_Kd,
            new_dd.roll_Kp, new_dd.roll_Ki, new_dd.roll_Kd,
            new_dd.yaw_Kp, new_dd.yaw_Ki, new_dd.yaw_Kd,
            new_dd.thrust_Kp, new_dd.thrust_Ki, new_dd.thrust_Kd,
        ]
        payload_buf = bytes()
        for val in payload:
            payload_buf += struct.pack('f', val)
        crc = calculate_modbus_crc(payload_buf)
        message = bytes([0b10101010, 0, len(payload_buf)]) + payload_buf + crc
        print(crc)
        print(message)
        self.tx_queue.append(message)

class MainWindow(QMainWindow):
    update_tunings= pyqtSignal(DroneData)

    def __init__(self):
        super(MainWindow, self).__init__()
        self.setWindowTitle("Drone GUI")
        self.dd = DroneData()

        self.raw_data = QLabel("No data")
        self.input_widget  = self.__create_input_widget()

        central_layout = QGridLayout()
        central_layout.addWidget(self.raw_data, 0, 0)
        central_layout.addWidget(self.input_widget, 1, 0)

        central_widget = QWidget()
        central_widget.setLayout(central_layout)
        self.setCentralWidget(central_widget)

        self.data_reader = DataReader()
        self.worker = Worker()
        self.reader_thread = QThread()

        self.worker.moveToThread(self.reader_thread)
        
        self.reader_thread.started.connect(self.worker.run)
        self.worker.progressed.connect(self.update_data)
        self.worker.finished.connect(self.reader_thread.quit)
        # self.update_tunings.connect(self.worker.update_tunings)
        self.update_tunings.connect(self.workerproxy)

        self.reader_thread.start()
        self.reset_pid_tunings()

    def workerproxy(self, dd):
        self.worker.update_tunings(dd)

    def apply_pid_tunings(self):
        try:
            dd = DroneData()
            dd.pitch_Kp = float(self.pitch_Kp_input.text())
            dd.pitch_Ki = float(self.pitch_Ki_input.text())
            dd.pitch_Kd = float(self.pitch_Kd_input.text())

            dd.roll_Kp = float(self.roll_Kp_input.text())
            dd.roll_Ki = float(self.roll_Ki_input.text())
            dd.roll_Kd = float(self.roll_Kd_input.text())

            dd.yaw_Kp = float(self.yaw_Kp_input.text())
            dd.yaw_Ki = float(self.yaw_Ki_input.text())
            dd.yaw_Kd = float(self.yaw_Kd_input.text())

            dd.thrust_Kp = float(self.thrust_Kp_input.text())
            dd.thrust_Ki = float(self.thrust_Ki_input.text())
            dd.thrust_Kd = float(self.thrust_Kd_input.text())

            self.update_tunings.emit(dd)
        except:
            print("Invalid input data")

    def reset_pid_tunings(self):
        self.pitch_Kp_input.setText(f"{self.dd.pitch_Kp:.5f}")
        self.pitch_Ki_input.setText(f"{self.dd.pitch_Ki:.5f}")
        self.pitch_Kd_input.setText(f"{self.dd.pitch_Kd:.5f}")

        self.roll_Kp_input.setText(f"{self.dd.roll_Kp:.5f}")
        self.roll_Ki_input.setText(f"{self.dd.roll_Ki:.5f}")
        self.roll_Kd_input.setText(f"{self.dd.roll_Kd:.5f}")

        self.yaw_Kp_input.setText(f"{self.dd.yaw_Kp:.5f}")
        self.yaw_Ki_input.setText(f"{self.dd.yaw_Ki:.5f}")
        self.yaw_Kd_input.setText(f"{self.dd.yaw_Kd:.5f}")

        self.thrust_Kp_input.setText(f"{self.dd.thrust_Kp:.5f}")
        self.thrust_Ki_input.setText(f"{self.dd.thrust_Ki:.5f}")
        self.thrust_Kd_input.setText(f"{self.dd.thrust_Kd:.5f}")
        pass

    def __create_input_widget(self):
        self.pitch_tunings_label = QLabel("Pitch P I D")
        self.pitch_Kp_input = QLineEdit()
        self.pitch_Ki_input = QLineEdit()
        self.pitch_Kd_input = QLineEdit()

        self.roll_tunings_label = QLabel("Roll P I D")
        self.roll_Kp_input = QLineEdit()
        self.roll_Ki_input = QLineEdit()
        self.roll_Kd_input = QLineEdit()

        self.yaw_tunings_label = QLabel("Yaw P I D")
        self.yaw_Kp_input = QLineEdit()
        self.yaw_Ki_input = QLineEdit()
        self.yaw_Kd_input = QLineEdit()

        self.thrust_tunings_label = QLabel("Thrust P I D")
        self.thrust_Kp_input = QLineEdit()
        self.thrust_Ki_input = QLineEdit()
        self.thrust_Kd_input = QLineEdit()

        self.apply_tunings_button = QPushButton("Apply")
        self.reset_tunings_button = QPushButton("Reset")
        self.apply_tunings_button.clicked.connect(self.apply_pid_tunings)
        self.reset_tunings_button.clicked.connect(self.reset_pid_tunings)

        input_widget_layout = QGridLayout()

        input_widget_layout.addWidget(self.pitch_tunings_label, 0, 0)
        input_widget_layout.addWidget(self.pitch_Kp_input, 0, 1)
        input_widget_layout.addWidget(self.pitch_Ki_input, 0, 2)
        input_widget_layout.addWidget(self.pitch_Kd_input, 0, 3)

        input_widget_layout.addWidget(self.roll_tunings_label, 1, 0)
        input_widget_layout.addWidget(self.roll_Kp_input, 1, 1)
        input_widget_layout.addWidget(self.roll_Ki_input, 1, 2)
        input_widget_layout.addWidget(self.roll_Kd_input, 1, 3)

        input_widget_layout.addWidget(self.yaw_tunings_label, 2, 0)
        input_widget_layout.addWidget(self.yaw_Kp_input, 2, 1)
        input_widget_layout.addWidget(self.yaw_Ki_input, 2, 2)
        input_widget_layout.addWidget(self.yaw_Kd_input, 2, 3)


        input_widget_layout.addWidget(self.thrust_tunings_label, 3, 0)
        input_widget_layout.addWidget(self.thrust_Kp_input, 3, 1)
        input_widget_layout.addWidget(self.thrust_Ki_input, 3, 2)
        input_widget_layout.addWidget(self.thrust_Kd_input, 3, 3)

        input_widget_layout.addWidget(self.apply_tunings_button, 4, 2)
        input_widget_layout.addWidget(self.reset_tunings_button, 4, 3)

        input_widget = QWidget()
        input_widget.setLayout(input_widget_layout)

        return input_widget
    


    def update_data(self, dd):
        self.dd = dd;
        self.raw_data.setText(
            f'''
Pitch {to_degrees(dd.Pitch):.2f}  Yaw {to_degrees(dd.Yaw):.2f}    Roll {to_degrees(dd.Roll):.2f}
Alt. {dd.Alt_relative:.2f}  Abs. {dd.Alt_absolute:.2f}   Radar {dd.Alt_radar:.2f} 
Setpoints:  Pitch: {to_degrees(dd.Pitch_sp):.2f}    Yaw: {to_degrees(dd.Yaw_sp):.2f}    Roll: {to_degrees(dd.Roll_sp):.2f}      Thrust: {to_degrees(dd.Thrust_sp):.2f} 
Motors: {dd.motors}
PID:
    Pitch   Kp: {dd.pitch_Kp:.5f} Ki: {dd.pitch_Ki:.5f} Kd: {dd.pitch_Kd:.5f}
    Roll    Kp: {dd.roll_Kp:.5f} Ki: {dd.roll_Ki:.5f} Kd: {dd.roll_Kd:.5f}
    Yaw     Kp: {dd.yaw_Kp:.5f} Ki: {dd.yaw_Ki:.5f} Kd: {dd.yaw_Kd:.5f}
    Thrust  Kp: {dd.thrust_Kp:.5f} Ki: {dd.thrust_Ki:.5f} Kd: {dd.thrust_Kd:.5f}
Last contact timestamp: {dd.last_response_time}
'''
        )
        

def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()

    sys.exit(app.exec())

if __name__ == "__main__":
    main()