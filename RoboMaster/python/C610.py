from usb_can_analyzer import Converter as can_analyzer
import threading
import serial.tools.list_ports
import time

## C610 brushless motor speed controller
class C610:
    def __init__(self, port=None):
        if port is None:
            self._hwid = "USB VID:PID=1A86:7523"
            self._baudrate = 2000000
            self._port = self._find_port_by_hwid()
            

        self.converter = can_analyzer(self._port, baudrate=self._baudrate)
        self.motor = {}

        self._send_request = False
        self._last_send_time = time.time()
        self.control_freq = 50

    def _communication(self):
        while True:
            try:
                data = self.converter.readMessage()
                if type(data) != int:
                    msg = data[2]

                    motor_id = data[1] - 0x200
                    if motor_id not in self.motor:
                        self.add_motor(motor_id)

                    pos = (msg[0] << 8) | msg[1]
                    self.motor[motor_id].pos = pos

                    vel = (msg[2] << 8) | msg[3]
                    if vel > 2**15:
                        vel -= 2**16
    
                    current = (msg[4] << 8) | msg[5]
                    if current > 2**15:
                        current -= 2**16
                    
                    self.motor[motor_id].vel = vel
                    self.motor[motor_id].current = current

                try:
                    if time.time() - self._last_send_time > 1/self.control_freq:
                        str_current = self._encode_to_hex(self.motor[1].target_current) + "000000"
                        self.converter.sendMessage(0, 0x200, str_current)
                        self._last_send_time = time.time()
                except:
                    print("Failed to send request")
                
            except KeyboardInterrupt:
                break
            except:
                pass
    
    def start_communication(self):
        self._communication_thread = threading.Thread(target=self._communication)
        self._communication_thread.start()

    def stop_communication(self):
        self._communication_thread.join()
        self.converter.close()

    def add_motor(self, motor_id):
        self.motor[motor_id] = M2006(motor_id)

    def set_current(self, motor_id, current):
        self.motor[motor_id].target_current = float(current) * 1000
        self._send_request = True
    
    def _find_port_by_hwid(self):
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if self._hwid in port.hwid:
                return port.device
        return None

    def get_pos(self):
        return self.pos
    
    def _encode_to_hex(self, value):
        value = int(value)
        if value < 0:
            value += 2**16
        hex_str = '{:04x}'.format(value)
        return hex_str
    
## M2006 brushless motor 
class M2006:
    def __init__(self, motor_id):
        self.motor_id = motor_id
        self.pos = 0
        self.vel = 0
        self.current = 0
        self.target_current = 0


if __name__ == "__main__":
    c610 = C610()
    c610.add_motor(1)
    c610.start_communication()
    while True:
        current = input("Enter current: ")
        c610.set_current(1, current)
    