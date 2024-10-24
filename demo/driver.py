#!/usr/bin/env python
import rospy
import serial
import time
import threading
from geometry_msgs.msg import Twist

MAX_INPUT = 5
WHEELBASE = 1
TIMEOUT_DURATION = 1.0  # Timeout duration in seconds

class MotorController:
    def __init__(self):
        rospy.init_node('motor_control', anonymous=True)
        self.serial_port = '/dev/ttyUSB0'
        self.baudrate = 9600
        self.timeout = 1
        self.error_count = 0
        self.max_errors = 5
        self.stop_thread = threading.Event()
        self.serial_lock = threading.Lock()  # To ensure thread-safe access to the serial port
        self.last_cmd_vel_time = rospy.Time.now()  # Track time of last cmd_vel
        self.initialize_serial()
        self.start_watchdog()

    def initialize_serial(self):
        with self.serial_lock:  # Lock to prevent multiple attempts to open the port
            if not hasattr(self, 'ser') or not self.ser.is_open:
                try:
                    self.ser = serial.Serial(self.serial_port, baudrate=self.baudrate, timeout=self.timeout)
                    rospy.loginfo("Serial connection established.")
                except Exception as e:
                    rospy.logerr(f"Failed to connect to serial: {e}")
                    time.sleep(0.5)  # Wait before trying to reconnect

    def reconnect_serial(self):
        with self.serial_lock:
            try:
                if self.ser.is_open:
                    self.ser.close()
                time.sleep(0.5)
                self.ser.open()
                rospy.loginfo("Reconnected to serial device.")
            except Exception as e:
                rospy.logerr(f"Reconnection attempt failed: {e}")
                time.sleep(0.5)

    def send_motor_command(self, address, command, value):
        with self.serial_lock:
            if self.ser.is_open:  # Only attempt to send if the port is open
                try:
                    packet = [address, command, value]
                    crc = self.crc16_xmodem(packet)
                    packet.append((crc >> 8) & 0xFF)
                    packet.append(crc & 0xFF)
                    self.ser.write(bytearray(packet))
                    rospy.loginfo(f"Sent packet: {packet}")
                    response = self.ser.read(1)
                    if response == b'\xFF':
                        self.error_count = 0
                    else:
                        rospy.logwarn(f"Unexpected response: {response}")
                        self.error_count += 1
                        if self.error_count >= self.max_errors:
                            self.reconnect_serial()
                            self.error_count = 0
                except Exception as e:
                    rospy.logerr(f"Error sending motor command: {e}")
                    self.reconnect_serial()

    def cmd_vel_callback(self, msg):
        self.last_cmd_vel_time = rospy.Time.now()
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        motor_controller_address = 0x80
        motor1_power = linear_x + (angular_z * WHEELBASE)
        motor2_power = -linear_x + (angular_z * WHEELBASE)
        motor1_power = int(self.normalize(motor1_power))
        motor2_power = int(self.normalize(motor2_power))
        if motor1_power >= 0:
            self.send_motor_command(motor_controller_address, 0x00, motor1_power)
        else:
            self.send_motor_command(motor_controller_address, 0x01, -motor1_power)
        if motor2_power >= 0:
            self.send_motor_command(motor_controller_address, 0x04, motor2_power)
        else:
            self.send_motor_command(motor_controller_address, 0x05, -motor2_power)

    def stop_motors(self):
        motor_controller_address = 0x80
        self.send_motor_command(motor_controller_address, 0x00, 0)
        self.send_motor_command(motor_controller_address, 0x04, 0)
        rospy.loginfo("Motors stopped due to timeout.")

    def watchdog_timer(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and not self.stop_thread.is_set():
            if (rospy.Time.now() - self.last_cmd_vel_time).to_sec() > TIMEOUT_DURATION:
                self.stop_motors()
            rate.sleep()

    def start_watchdog(self):
        watchdog_thread = threading.Thread(target=self.watchdog_timer)
        watchdog_thread.daemon = True
        watchdog_thread.start()

    def motor_control_node(self):
        
        rospy.Subscriber('/move_base/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.spin()
        self.stop_thread.set()
        with self.serial_lock:
            if self.ser.is_open:
                self.ser.close()

    def normalize(self, value):
        max_input = MAX_INPUT
        max_output = 127
        normalized_value = (value / max_input) * max_output
        return max(-max_output, min(normalized_value, max_output))

    def crc16_xmodem(self, packet):
        crc = 0x0000
        for byte in packet:
            crc ^= (byte << 8)
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ 0x1021
                else:
                    crc <<= 1
            crc &= 0xFFFF
        return crc

if __name__ == '__main__':
    try:
        motor_controller = MotorController()
        motor_controller.motor_control_node()
    except rospy.ROSInterruptException:
        pass
