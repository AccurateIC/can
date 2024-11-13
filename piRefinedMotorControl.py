#!/usr/bin/env python
import can
import math
import serial
import time
import threading
import atexit
import rospy
from std_msgs.msg import Float64  # Import Float64 message type




class MotorController:
    # Define class-level constants for configurability
    DEFAULT_CAN_INTERFACE = 'can0'
    DEFAULT_CAN_ID = 0x0C07BDA7
    PWM_MIN_OLD = 950
    PWM_MAX_OLD = 2000
    PWM_MIN_NEW = 0
    PWM_MAX_NEW = 1500
    RPM_BASE = 32000
    RPM_MAX = 65535

    PWM_MIN_DIR_OLD = 860
    PWM_MAX_DIR_OLD = 1880
    DIR_MIN_NEW = -500
    DIR_MAX_NEW = 500
    TOLERANCE = 75
    
    GEAR_FORWARD = 1200
    
    GEAR_REVERSE = 1500


    def __init__(self):
        # Initialize RPM value
        rospy.init_node('driver', anonymous=True)
        self.direction_publisher = rospy.Publisher('/steering_motor_cmd', Float64, queue_size=10)
        self.scaled_direction_rpm = 0
        self.scaledrpm_value = 0

        self.float64_msg = Float64()
        self.gear_flag = 0
        # Set up the CAN interface with error handling
        try:
            self.bus = can.interface.Bus(channel=self.DEFAULT_CAN_INTERFACE, interface='socketcan')
            # print(f'Initialized CAN bus on {self.DEFAULT_CAN_INTERFACE}')
        except Exception as e:
            # print(f'Failed to initialize CAN bus on {self.DEFAULT_CAN_INTERFACE}: {e}')
            exit(1)

        # Start a separate thread to receive data from the serial port
        self.serial_thread = threading.Thread(target=self.receive_data)
        self.serial_thread.daemon = True  # Daemonize thread to allow program exit
        self.serial_thread.start()

        # Register shutdown hook for cleanup
        atexit.register(self.shutdown_hook)

        # Set up a timer to publish CAN messages at 100 Hz
        self.publish_timer = threading.Event()
        threading.Timer(0.01, self.publish_can_message).start()  # 10ms

    def scale_value(self, value):
        """
        Scales the input PWM value from the old range to the new range.
        """
        try:
            value_int = int(value)
            # print(value, type(value), value_int, type(value_int))
        except ValueError:
            # print(f'Invalid PWM value type: {value}, {type(value)}')
            return 0

        if value_int < self.PWM_MIN_OLD:
            # print(f'PWM value {value_int} below min_old {self.PWM_MIN_OLD}. Scaling to {self.PWM_MIN_NEW}.')
            return self.PWM_MIN_NEW
        elif value_int > self.PWM_MAX_OLD:
            # print(f'PWM value {value_int} exceeds max_old {self.PWM_MAX_OLD}. Clipping to {self.PWM_MAX_NEW}.')
            return self.PWM_MAX_NEW
        else:
            scaled = ((value_int - self.PWM_MIN_OLD) / (self.PWM_MAX_OLD - self.PWM_MIN_OLD)) * (self.PWM_MAX_NEW - self.PWM_MIN_NEW) + self.PWM_MIN_NEW
            # print(f'Scaled PWM value: {scaled}')
            return scaled
    
    def direction_scaled(self, direction_pwm):
        """
        Scales the direction PWM value based on its respective range,
        with tolerance for zero output at the midpoint ± 50.
        """
        try:
            direction_pwm_int = int(direction_pwm)
        except ValueError:
            # print(f'Invalid direction PWM value: {direction_pwm}')
            return 0

        # Calculate the midpoint of the PWM range
        pwm_midpoint = (self.PWM_MIN_DIR_OLD + self.PWM_MAX_DIR_OLD) / 2

        # Apply tolerance for direction
        if (pwm_midpoint - self.TOLERANCE) <= direction_pwm_int <= (pwm_midpoint + self.TOLERANCE):
            # print(f'Direction PWM {direction_pwm_int} within tolerance range. Setting scaled direction to 0.')
            return 0
        elif direction_pwm_int < self.PWM_MIN_DIR_OLD:
            # print(f'Direction PWM {direction_pwm_int} below min_old {self.PWM_MIN_DIR_OLD}. Scaling to {self.DIR_MIN_NEW}.')
            return self.DIR_MIN_NEW
        elif direction_pwm_int > self.PWM_MAX_DIR_OLD:
            # print(f'Direction PWM {direction_pwm_int} exceeds max_old {self.PWM_MAX_DIR_OLD}. Clipping to {self.DIR_MAX_NEW}.')
            return self.DIR_MAX_NEW
        else:
            # Scaling the direction PWM value
            scaled_direction = ((direction_pwm_int - self.PWM_MIN_DIR_OLD) / (self.PWM_MAX_DIR_OLD - self.PWM_MIN_DIR_OLD)) * (self.DIR_MAX_NEW - self.DIR_MIN_NEW) + self.DIR_MIN_NEW
            # print(f'Scaled direction value: {scaled_direction}')
            return scaled_direction
    
    def set_gear_flag(self,input_value):
        inp = int(input_value)

        # Compare input_value with GEAR_FORWARD and GEAR_REVERSE
        if inp > self.GEAR_REVERSE:
            self.gear_flag = -1
        elif inp < self.GEAR_FORWARD:
            self.gear_flag = 1
        else:
            self.gear_flag = 0

        

    def receive_data(self):
        """
        Listens for incoming data on the serial port.
        """
        # Open the serial port (replace ttyTHS0 with your UART port)
        ser = serial.Serial('/dev/ttyTHS0', 115200, timeout=1)

        # Give some time for the connection to be established
        time.sleep(2)

        while True:
            if ser.in_waiting > 0:
                # Read incoming data from the serial port
                data = ser.readline().decode().strip()
                # print(data)
                
                if data.startswith("DIR"):
                    # print(f"Recieved: {data}")

                    self.scaled_direction_rpm = self.direction_scaled(float(data[3:]))
                    # print(f"Updated Direction RPM value: {self.scaled_direction_rpm}")
                    self.float64_msg.data = -self.scaled_direction_rpm
        
                    # Log the current value of linear_x for debugging
                    # rospy.loginfo(f"Publishing Float64 message with data = {self.scaled_direction_rpm}")
        
                    # Publish the message
                    self.direction_publisher.publish(self.float64_msg)

                elif data.startswith("GEA"):
                    # print(f"Recieved: {data}")
                    self.set_gear_flag(float(data[3:]))
                    

                elif data.startswith("THR"):
                    # print(f"Received: {data}, and gear_flag= {self.gear_flag}")
                    # Scale and update the RPM value

                    
                    scaled_rpm = self.scale_value(float(data[3:])) 
                    self.scaledrpm_value = int(scaled_rpm) 
                    print(f'Updated scaled RPM value: {self.scaledrpm_value}')
                    self.publish_can_message()


    def publish_can_message(self):
        """
        Publishes the scaled RPM value as a CAN message.
        """
        if self.scaledrpm_value != 0:
            adjusted_rpm = math.floor(self.scaledrpm_value) + self.RPM_BASE

            # Clip to 16-bit range
            self.rpm_value = min(max(adjusted_rpm, 0), self.RPM_MAX)
            low_byte = self.rpm_value & 0xFF
            high_byte = (self.rpm_value >> 8) & 0xFF
            
            # Prepare the CAN message
            data = [0x00, 0x7D, low_byte, high_byte, 0xF0, 0x00, 0x00, 0x00]
            msg = can.Message(arbitration_id=self.DEFAULT_CAN_ID, data=data, is_extended_id=True)

            try:
                self.bus.send(msg)
                print(f'CAN message sent: {msg}')
            except can.CanError as e:
                print(f'Error sending CAN message: {e}')
        else:
            data = [0x00, 0x7D, 0x00, 0x7D, 0xF0, 0x00, 0x00, 0x00]
            msg = can.Message(arbitration_id=self.DEFAULT_CAN_ID, data=data)
            print('Scaled RPM value is zero; sending 0 rpm CAN message.')

    def shutdown_hook(self):
        """
        Cleanup function to be called on program exit.
        """
        try:
            self.bus.shutdown()
            print('CAN bus shutdown successfully.')
        except AttributeError:
            print('CAN bus was not initialized; nothing to shutdown.')
        except Exception as e:
            print(f'Error during CAN bus shutdown: {e}')

if __name__ == '__main__':
    try:
        controller = MotorController()
        while not rospy.is_shutdown():
            time.sleep(0.1)  # Keep the main thread alive
    except KeyboardInterrupt:
        print("Shutting down.")
        exit()
