#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import serial
import threading
import time
import serial
import rospy 
import sys
import struct as st
import binascii

from time import time
from sensor_msgs.msg import Imu, Temperature, MagneticField

from tf.transformations import quaternion_from_euler
from dynamic_reconfigure.server import Server
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

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
        
        return self.__integral + self.kp*error + d


class Modbus:
    def __init__(self):
        self.motor1_power = 50
        self.motor2_power = 50
        self.motor3_power = 50
        self.motor4_power = 50
        self.ser = serial.Serial('/dev/ttyACM0', 115200)  # Adjust to your USB port
        self.last_cmd_vel = None  # To store the last cmd_vel message
        self.stop_thread = threading.Event()  # Event to signal the thread to stop
        self.read_thread = threading.Thread(target=self.read_modbus_periodically)
        self.read_thread.daemon = True
        self.read_thread.start()
        rospy.init_node('cmd_vel_to_stm32', anonymous=True)


        self.pub_data = rospy.Publisher('imu/data', Imu, queue_size=1)
        self.pub_raw = rospy.Publisher('imu/raw', Imu, queue_size=1)
        self.pub_mag = rospy.Publisher('imu/mag', MagneticField, queue_size=1)
        self.pub_temp = rospy.Publisher('imu/temp', Temperature, queue_size=1)


        self.port = rospy.get_param('~port', '/dev/ttyUSB0')
        self.frame_id = rospy.get_param('~frame_id', 'imu_link')
        self.frequency = rospy.get_param('frequency', 100)
        self.operation_mode = rospy.get_param('operation_mode', OPER_MODE_NDOF)
        self.publish_mag = rospy.get_param('~publish_mag', False)
        self.publish_raw = rospy.get_param('~publish_raw', False)
        try:
            self.listener()
        except rospy.ROSInterruptException:
            pass
        finally:
            self.stop_thread.set()
            self.read_thread.join()
            self.ser.close()

    def callback(self, data):
        if (self.last_cmd_vel is None or data.linear.x != self.last_cmd_vel.linear.x or data.angular.z != self.last_cmd_vel.angular.z) : 
            self.last_cmd_vel = data
            self.interpret(data.linear.x, data.angular.z)
            print("Motor 1 Byte:", self.motor1_power)
            print("Motor 2 Byte:", self.motor2_power)
            print("Motor 3 Byte:", self.motor3_power)
            print("Motor 4 Byte:", self.motor4_power)
            self.multiple_write_command(self.motor1_power,self.motor2_power,self.motor3_power,self.motor4_power)
            


            #self.write_modbus_command(self.motor1_power)

    def listener(self):
        rospy.Subscriber("/cmd_vel", Twist, self.callback)
        rospy.spin()

    def interpret(self, linear_x, angular_z):
        wheelbase = 1.0  # distance between left and right wheels

        # Calculate motor speeds for differential drive
        self.motor1_power = linear_x - (angular_z * wheelbase / 2)+2.5  # front right motor
        self.motor2_power = linear_x + (angular_z * wheelbase / 2)+2.5  # front left motor
        self.motor3_power = linear_x - (angular_z * wheelbase / 2)+2.5  # rear right motor
        self.motor4_power = linear_x + (angular_z * wheelbase / 2)+2.5 # rear left motor

        # Normalize motor speeds to be within the range -100 to 100
        self.motor1_power = self.normalize(self.motor1_power)
        self.motor2_power = self.normalize(self.motor2_power)
        self.motor3_power = self.normalize(self.motor3_power)
        self.motor4_power = self.normalize(self.motor4_power)




    def multiple_write_command(self, motor1, motor2, motor3, motor4_power):
        command = [0x01, 0x10, 0x00, 0x00, 0x00, 0x04, 0x08]  # Preset Multiple Registers command
        max_pwm = 300.0
        factor = max_pwm/256.0
        
        motor1_value = abs(int(motor1 * 65535 / 25600*factor))  # Scaling motor1 to 0-65535 range
        motor2_value = abs(int(motor2 * 65535 / 25600*factor))  # Scaling motor2 to 0-65535 range
        motor3_value = abs(int(motor3 * 65535 / 25600*factor))  # Scaling motor3 to 0-65535 range
        motor4_value = abs(int(motor4_power * 65535 / 25600*factor))  # Scaling motor4_power to 0-65535 range

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
        command.append(motor1_high_byte)
        command.append(motor1_low_byte)
        command.append(motor2_high_byte)
        command.append(motor2_low_byte)
        command.append(motor3_high_byte)
        command.append(motor3_low_byte)
        command.append(motor4_high_byte)
        command.append(motor4_low_byte)

        # Calculate and append the CRC
        crc_low, crc_high = self.get_crc_low_high(command)
        command.append(crc_low)
        command.append(crc_high)

        # Convert the command list to a byte array
        command_bytes = bytearray(command)
        self.ser.write(command_bytes)
        # Debug: Print the command to be sent
        print("Write Multiple Modbus Command:", [f"{byte:02X}" for byte in command_bytes])

        # Send the command
        

        # Read the response
        # response = self.ser.read(8)  # Response for Write Multiple Registers is 8 bytes
        # print("Write Multiple Modbus Response:", [f"{byte:02X}" for byte in response])


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
        max_input = 5.0  # maximum value of linear_x or angular_z
        max_output = 100  # desired max output range
        normalized_value = (value / max_input) * max_output
        if normalized_value > 100:
            normalized_value = 100
        elif normalized_value < 0:
            normalized_value = 0
        return normalized_value
    def read_modbus_command(self):
        command = [0x01, 0x03, 0x00, 0x00, 0x00, 0x50]  # Read Holding Registers command
        low_byte, high_byte = self.get_crc_low_high(command)
        command.append(low_byte)
        command.append(high_byte)

        # Convert the command list to a byte array
        command_bytes = bytearray(command)
        self.ser.write(command_bytes)

        # Read the first 3 bytes to get the header
        header = self.ser.read(3)
        if len(header) < 3:
            raise Exception("Failed to read the Modbus response header")

        # Parse the header to determine the length of the remaining response
        byte_count = header[2]

        
        remaining_bytes = self.ser.read(byte_count + 2)
        if len(remaining_bytes) < (byte_count + 2):
            raise Exception("Failed to read the full Modbus response")

        
        response = header + remaining_bytes

        print("Read Modbus Response:", response)
        return remaining_bytes


    """ 
    def read_modbus_command(self):
        command = [0x01, 0x03, 0x00, 0x00, 0x00, 0x04  # Read Holding Registers command
        low_byte, high_byte = self.get_crc_low_high(command)
        command.append(low_byte)
        command.append(high_byte)

        # Convert the command list to a byte array
        command_bytes = bytearray(command)
        self.ser.write(command_bytes)
        response = self.ser.read(13) 
     
        print("Read Modbus Response:",response)"""
        
    def write_modbus_command(self, motor1):
        command = [0x01, 0x06, 0x00, 0x00]  # Write Single Register command
        
        # Convert motor1 to an appropriate integer value within the range of 0-65535
        motor1_value = abs(int(motor1 * 65535 / 25600)) # Scaling motor1 to 0-65535 range
        motor1_value = motor1_value*2
        # Extract high and low bytes
        high_byte = (motor1_value >> 8) & 0xFF
        low_byte = motor1_value & 0xFF
        
        # Append the high and low bytes to the command
        command.append(high_byte)
        command.append(low_byte)
        
        # Calculate and append the CRC
        crc_low, crc_high = self.get_crc_low_high(command)
        command.append(crc_low)
        command.append(crc_high)

        # Convert the command list to a byte array
        command_bytes = bytearray(command)
        
        # Debug: Print the command to be sent
        print("Write Modbus Command:", [f"{byte:02X}" for byte in command_bytes])

        # Send the command
        self.ser.write(command_bytes)

        # Read the response


    def read_modbus_periodically(self):
        while not self.stop_thread.is_set():
            response = self.read_modbus_command()
            imu_callback(response)
            rate.sleep()
            time.sleep(0.5)  # Adjust the sleep time as needed

    def imu_callback(self,buf):
        q_fact = 16384.0
        acc_fact = 100.0
        mag_fact = 16.0
        gyr_fact = 900.0
        seq = 0
        if buf != 0:
            # Publish raw data
            self.imu_raw.header.stamp = rospy.Time.now()
            self.imu_raw.header.frame_id = frame_id
            self.imu_raw.header.seq = seq
            self.imu_raw.orientation_covariance[0] = -1
            self.imu_raw.linear_acceleration.x = float(st.unpack('h', st.pack('BB', buf[0], buf[1]))[0]) / acc_fact
            self.imu_raw.linear_acceleration.y = float(st.unpack('h', st.pack('BB', buf[2], buf[3]))[0]) / acc_fact
            self.imu_raw.linear_acceleration.z = float(st.unpack('h', st.pack('BB', buf[4], buf[5]))[0]) / acc_fact
            self.imu_raw.linear_acceleration_covariance[0] = -1
            self.imu_raw.angular_velocity.x = float(st.unpack('h', st.pack('BB', buf[12], buf[13]))[0]) / gyr_fact
            self.imu_raw.angular_velocity.y = float(st.unpack('h', st.pack('BB', buf[14], buf[15]))[0]) / gyr_fact
            self.imu_raw.angular_velocity.z = float(st.unpack('h', st.pack('BB', buf[16], buf[17]))[0]) / gyr_fact
            self.imu_raw.angular_velocity_covariance[0] = -1
            self.pub_raw.publish(self.imu_raw)

        #            print("read: ", binascii.hexlify(buf), "acc = (",self.imu_data.linear_acceleration.x,
        #                  self.imu_data.linear_acceleration.y, self.imu_data.linear_acceleration.z, ")")

        # Publish filtered data
            self.imu_data.header.stamp = rospy.Time.now()
            self.imu_data.header.frame_id = self.frame_id
            self.imu_data.header.seq = seq
            self.imu_data.orientation_covariance = [0.0159,0,0,0,0.0159,0,0,0,0.0159]
            self.imu_data.orientation.w = float(st.unpack('h', st.pack('BB', buf[24], buf[25]))[0]) / q_fact
            self.imu_data.orientation.x = float(st.unpack('h', st.pack('BB', buf[26], buf[27]))[0]) / q_fact
            self.imu_data.orientation.y = float(st.unpack('h', st.pack('BB', buf[28], buf[29]))[0]) / q_fact
            self.imu_data.orientation.z = float(st.unpack('h', st.pack('BB', buf[30], buf[31]))[0]) / q_fact
            self.imu_data.linear_acceleration.x = float(st.unpack('h', st.pack('BB', buf[32], buf[33]))[0]) / acc_fact
            self.imu_data.linear_acceleration.y = float(st.unpack('h', st.pack('BB', buf[34], buf[35]))[0]) / acc_fact
            self.imu_data.linear_acceleration.z = float(st.unpack('h', st.pack('BB', buf[36], buf[37]))[0]) / acc_fact
            self.imu_data.linear_acceleration_covariance = [0.017,0,0,0,0.017,0,0,0,0.017]
            self.imu_data.angular_velocity.x = float(st.unpack('h', st.pack('BB', buf[12], buf[13]))[0]) / gyr_fact
            self.imu_data.angular_velocity.y = float(st.unpack('h', st.pack('BB', buf[14], buf[15]))[0]) / gyr_fact
            self.imu_data.angular_velocity.z = float(st.unpack('h', st.pack('BB', buf[16], buf[17]))[0]) / gyr_fact
            self.imu_data.angular_velocity_covariance = [0.04,0,0,0,0.04,0,0,0,0.04]
            self.pub_data.publish(self.imu_data)

            # Publish magnetometer data
            if publish_mag:
                mag_msg.header.stamp = rospy.Time.now()
                mag_msg.header.frame_id = frame_id
                mag_msg.header.seq = seq
                mag_msg.magnetic_field.x = float(st.unpack('h', st.pack('BB', buf[6], buf[7]))[0]) / mag_fact
                mag_msg.magnetic_field.y = float(st.unpack('h', st.pack('BB', buf[8], buf[9]))[0]) / mag_fact
                mag_msg.magnetic_field.z = float(st.unpack('h', st.pack('BB', buf[10], buf[11]))[0]) / mag_fact
                pub_mag.publish(mag_msg)

            # Publish temperature
            temperature_msg.header.stamp = rospy.Time.now()
            temperature_msg.header.frame_id = frame_id
            temperature_msg.header.seq = seq
            temperature_msg.temperature = buf[44]
            pub_temp.publish(temperature_msg)

            #yaw = float(st.unpack('h', st.pack('BB', buf[18], buf[19]))[0]) / 16.0
            #roll = float(st.unpack('h', st.pack('BB', buf[20], buf[21]))[0]) / 16.0
            #pitch = float(st.unpack('h', st.pack('BB', buf[22], buf[23]))[0]) / 16.0
            #           print "RPY=(%.2f %.2f %.2f)" %(roll, pitch, yaw)

            seq = seq + 1
    
if __name__ == "__main__":
    modbus = Modbus()

