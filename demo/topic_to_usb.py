#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import serial
import threading
import time
import socket
import json

crcTable = [
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
]

MAX_INPUT = 5
WHEELBASE = 1 
LIMIT= 900
MIN_PWM=50
MIN_PWM_ANGULAR=30


class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.__integral = 0.0
        self.prev_error = 0.0

    def update(self, actual, target, dt):
        error = target - actual
        d = self.kd * (error - self.prev_error) / dt
        self.__integral += self.ki * error * dt
        self.prev_error = error

        return self.__integral + self.kp * error + d

class Modbus:
    
    def __init__(self):
        rospy.loginfo("Initializing Modbus...")
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.ip_address = 'localhost'
        self.port = 10001
        self.max_attempts = 1000  # Maximum number of port increments to try
        self.ser = None 
        rospy.init_node('cmd_vel_to_stm32', anonymous=True)
        bound = False
        attempts = 0

        while not bound and attempts < self.max_attempts:
            try:
                self.server_socket.bind((self.ip_address, self.port))
                bound = True
                rospy.loginfo("\n" + "="*50)
                rospy.loginfo(f"=== SERVER SOCKET SUCCESSFULLY BOUND TO IP: {self.ip_address} ON PORT {self.port} ===")
                rospy.loginfo("="*50 + "\n")
            except OSError as e:
                if e.errno == 98:  # Address already in use
                    rospy.logwarn(f"Port {self.port} already in use. Trying port {self.port + 1}")
                    self.port += 1
                    attempts += 1
                else:
                    rospy.logerr(f"Failed to bind server socket: {e}")
                    raise

        if not bound:
            rospy.logerr(f"Failed to bind the server socket after {self.max_attempts} attempts. Exiting.")
            raise Exception("Unable to bind to a free port.")

        self.server_socket.listen(5)
        self.server_socket.settimeout(0.1)

        # Initialize the serial connection here
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # Adjust to your USB port
            rospy.loginfo("Device connected")
        except serial.SerialException as e:
            rospy.logerr(f"Failed to connect to the serial device: {e}")
            raise
        self.server_socket.listen(5)
        self.server_socket.settimeout(0.1)
        self.motor1_power = 0
        self.motor2_power = 0
        self.motor3_power = 0
        self.motor4_power = 0
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # Adjust to your USB port
        rospy.loginfo("Device connected")
        self.last_cmd_vel = None  # To store the last cmd_vel message
        self.stop_thread = threading.Event()  # Event to signal the thread to stop
        self.read_thread = threading.Thread(target=self.read_modbus_periodically)
        self.emerge_stop_thread = threading.Thread(target=self.tcp_thread)
        self.periodic_write_thread = threading.Thread(target=self.write_thread)
        self.read_thread.daemon = True
        self.read_thread.start()
        self.error_count = 0
        self.max_errors = 4
        self.emerge_stop_thread.start()
        #self.periodic_write_thread.start()
        self.intiliza_params()
        self.cmd_vel_timer = None

        try:
            self.listener()
        except rospy.ROSInterruptException:
            pass
        finally:
            self.stop_thread.set()
            self.read_thread.join()
            self.ser.close()
    def intiliza_params(self):
        self.write_modbus_command(15,LIMIT)
        self.write_modbus_command(16,MIN_PWM)
        self.write_modbus_command(17,MIN_PWM_ANGULAR)

    def write_modbus_command(self, address,register):
        command = [0x01, 0x06]  # Write Single Register command

        # Append the address as two bytes (high and low bytes)
        command.append((address >> 8) & 0xFF)  # High byte of address
        command.append(address & 0xFF)  # Low byte of address

        # Convert motor1 to an appropriate integer value within the range of 0-65535
        register_value = register # Scaling motor1 to 0-65535 range

        # Extract high and low bytes of motor1 value
        high_byte = (register_value >> 8) & 0xFF
        low_byte =register_value & 0xFF

        # Append the high and low bytes to the command
        command.append(high_byte)
        command.append(low_byte)

        # Calculate and append the CRC
        crc_low, crc_high = self.get_crc_low_high(command)
        command.append(crc_low)
        command.append(crc_high)

        # Convert the command list to a byte array
        command_bytes = bytearray(command)

        rospy.loginfo(f"Write Modbus Command: {[f'{byte:02X}' for byte in command_bytes]}")

        # Send the command
        self.ser.write(command_bytes)
    def callback(self, data):
        if self.cmd_vel_timer:
            self.cmd_vel_timer.cancel()
        self.cmd_vel_timer = threading.Timer(1.0, self.cmd_vel_timeout)
        self.cmd_vel_timer.start()

        if self.last_cmd_vel is None or data.linear.x != self.last_cmd_vel.linear.x or data.angular.z != self.last_cmd_vel.angular.z:
            self.last_cmd_vel = data
            self.interpret(data.linear.x, data.angular.z)
            #rospy.loginfo(f"Motor 1 Byte: {self.motor1_power}")
            #rospy.loginfo(f"Motor 2 Byte: {self.motor2_power}")
            #rospy.loginfo(f"Motor 3 Byte: {self.motor3_power}")
            #rospy.loginfo(f"Motor 4 Byte: {self.motor4_power}")
            self.multiple_write_command(self.motor1_power, self.motor2_power,self.motor3_power, self.motor4_power)
        if data.linear.x == 0 and data.angular.z == 0:
            self.multiple_write_command(0, 0, 0,0)
            
    def listener(self):
        rospy.Subscriber("/move_base/cmd_vel", Twist, self.callback)
        rospy.spin()

    def cmd_vel_timeout(self):
        rospy.logwarn("No cmd_vel message received for 1 second. Stopping motors.")
        self.motor1_power = 0
        self.motor2_power = 0
        self.motor3_power = 0
        self.motor4_power = 0
        self.multiple_write_command(self.motor1_power, self.motor2_power, self.motor3_power, self.motor4_power)

    def interpret(self, linear_x, angular_z):
         # distance between left and right wheels

        # Calculate motor speeds for differential drive
        if angular_z> 0:    
            self.motor1_power = -linear_x + (2/3*angular_z * WHEELBASE / 2)   # front right motor
            self.motor2_power = -linear_x - (4/3*angular_z * WHEELBASE / 2)  # front left motor
        else:    
            self.motor1_power = -linear_x + (4/3*angular_z * WHEELBASE / 2)   # front right motor
            self.motor2_power = -linear_x - (2/3*angular_z * WHEELBASE / 2)  # front left motor
        #self.motor3_power = 0#linear_x - (angular_z * WHEELBASE / 2)  # rear right motor
        #self.motor4_power = 0#-linear_x - (angular_z * WHEELBASE / 2)# rear left motor

        # Normalize motor speeds to be within the range -100 to 100
        self.motor1_power = self.normalize(self.motor1_power)
        self.motor2_power = self.normalize(self.motor2_power)
        self.motor3_power = self.normalize(self.motor3_power)
        self.motor4_power = self.normalize(self.motor4_power)

    def multiple_write_command(self, motor1, motor2, motor3, motor4_power):
        command = [0x01, 0x10, 0x00, 0x00, 0x00, 0x04, 0x08]  # Preset Multiple Registers command
        max_pwm = LIMIT
        factor = max_pwm / 256.0

        motor1_value = (int(motor1 * 65535 / 25600 * factor))  # Scaling motor1 to 0-65535 range
        motor2_value = (int(motor2 * 65535 / 25600 * factor))  # Scaling motor2 to 0-65535 range
        motor3_value = (int(motor3 * 65535 / 25600 * factor))  # Scaling motor3 to 0-65535 range
        motor4_value = (int(motor4_power * 65535 / 25600 * factor))  # Scaling motor4_power to 0-65535 range

        # Extract high and low bytes for each motor value
        motor1_high_byte = (motor1_value >> 8) & 0xFF
        motor1_low_byte = motor1_value & 0xFF
        motor2_high_byte = (motor2_value >> 8) & 0xFF
        motor2_low_byte = motor2_value & 0xFF
        motor3_high_byte = (motor3_value >> 8) & 0xFF
        motor3_low_byte = motor3_value & 0xFF
        motor4_high_byte = (motor4_value >> 8) & 0xFF
        motor4_low_byte = motor4_value & 0xFF

        # Append the high and low bytes to the command
        command += [
            motor1_high_byte, motor1_low_byte,
            motor2_high_byte, motor2_low_byte,
            motor3_high_byte, motor3_low_byte,
            motor4_high_byte, motor4_low_byte
        ]

        # Calculate and append the CRC
        crc_low, crc_high = self.get_crc_low_high(command)
        command.append(crc_low)
        command.append(crc_high)

        # Convert the command list to a byte array
        command_bytes = bytearray(command)
        self.ser.write(command_bytes)

        #rospy.loginfo(f"Write Multiple Modbus Command: {[f'{byte:02X}' for byte in command_bytes]}")

    def get_crc_low_high(self, buffer):
        crc = self.calculate_crc(buffer)
        low_byte = crc & 0xFF
        high_byte = (crc >> 8) & 0xFF
        return low_byte, high_byte

    def calculate_crc(self, buffer):
        crc = 0xFFFF
        for byte in buffer:
            crc = (crc >> 8) ^ crcTable[(crc & 0xFF) ^ byte]
        return crc

    def normalize(self, value):
        max_input = MAX_INPUT  # maximum value of linear_x or angular_z
        max_output = 100  # desired max output range
        normalized_value = (value / max_input) * max_output
        if normalized_value > 100:
            normalized_value = 100
        elif normalized_value < -100:
            normalized_value = -100
        return normalized_value
    def read_modbus_command(self, number):
        command = [0x01, 0x03, 0x00, 0x00, 0x00, number]  # Read Holding Registers command
        low_byte, high_byte = self.get_crc_low_high(command)
        command.append(low_byte)
        command.append(high_byte)

        # Convert the command list to a byte array
        command_bytes = bytearray(command)
        self.ser.write(command_bytes)
        
        # Read the header and check if the length is correct
        header = self.ser.read(3)
        header = [f"{byte:02X}" for byte in header]
        if len(header) < 3:
            raise Exception("Failed to read the Modbus response header")
        
        # Read the response based on the length provided in the header
        response = self.ser.read(int(header[2], 16) + 2)
        response = [f"{byte:02X}" for byte in response[:-2]]  # Exclude CRC bytes
        
        # Convert the response to signed integers (two bytes per integer)
        int_list = []
        for i in range(0, len(response), 2):
            unsigned_value = int(response[i] + response[i + 1], 16)
            # Check if the value should be interpreted as signed (based on most significant bit)
            if unsigned_value > 0x7FFF:  # If the value is greater than the maximum positive value for a 16-bit signed int
                signed_value = unsigned_value - 0x10000  # Convert to signed
            else:
                signed_value = unsigned_value
            int_list.append(signed_value)

        rospy.loginfo(f"MODBUS: {int_list}")
        rospy.loginfo(f"MOTOR1 = {int_list[4]} MOTOR2 = {int_list[5]} MOTOR3 = {-int_list[6]} MOTOR4 = {-int_list[7]}")





    def shutdown_callback(self):
        rospy.loginfo("Shutting down node...")

    def read_modbus_periodically(self):
        while not self.stop_thread.is_set():
            try:
                self.read_modbus_command(20)
                self.error_count = 0  # Reset the error counter on successful read
                time.sleep(0.08)  # Adjust sleep time as needed
            except Exception as e:
                rospy.logerr(f"Error in read thread: {e}")

                # Check for specific error messages
                if 'list index out of range' in str(e):
                    self.error_count += 1
                    if self.error_count > self.max_errors:
                        self.reconnect_serial()
                        self.error_count = 0  # Reset error counter
                elif 'Input/output error' in str(e):
                    self.reconnect_serial()
                    self.error_count = 0

    def reconnect_serial(self):
        while not self.stop_thread.is_set():
            try:
                self.ser.close()
                time.sleep(0.5)  # Wait before trying to reconnect
                self.ser.open()
                rospy.loginfo("Reconnected to serial device.")
                self.intiliza_params()
                return
            except Exception as e:
                rospy.logerr(f"Reconnection attempt failed: {e}")
                time.sleep(0.5)

    def tcp_thread(self):
        while not self.stop_thread.is_set():
            try:
                self.tcp_listen()
                time.sleep(0.1)  # Adjust sleep time as needed
            except Exception as e:
                rospy.logerr(f"Error in TCP thread: {e}")

    def write_thread(self):
        while not self.stop_thread.is_set():
            try:
                rospy.loginfo(f"PERİODİC WRİTE")
                self.multiple_write_command(self.motor1_power, self.motor2_power, self.motor3_power, self.motor4_power)
                time.sleep(5)  # Adjust sleep time as needed
            except Exception as e:
                rospy.logerr(f"Error in write thread: {e}")

    def tcp_listen(self):
        try:
            client_socket, client_address = self.server_socket.accept()
            rospy.loginfo(f"Connection from {client_address}")

            data = client_socket.recv(1024)
            if data:
                json_data = data.decode('utf-8')
                parsed_data = json.loads(json_data)

                if parsed_data.get('stop', False):
                    self.multiple_write_command(0, 0, 0, 0)
                    rospy.logwarn("**************  EMERGENCY_STOP  *****************")
                    rospy.logwarn("**************  EMERGENCY_STOP  *****************")
                    rospy.logwarn("**************  EMERGENCY_STOP  *****************")
                    
                    

        except socket.timeout:
            pass

if __name__ == "__main__":
    modbus = Modbus()
