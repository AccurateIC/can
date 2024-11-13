#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
import can
import math

class MotorController:
    # Define class-level constants for configurability
    DEFAULT_CAN_INTERFACE = 'can0'
    DEFAULT_CAN_ID = 0x0C07BDA7
    PWM_MIN_OLD = 950
    PWM_MAX_OLD = 2000
    PWM_MIN_NEW = 0
    PWM_MAX_NEW = 1000
    RPM_BASE = 32000
    RPM_MAX = 65535

    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('motor_controller', anonymous=True)

        # Retrieve parameters from ROS parameter server or use defaults
        self.can_interface = rospy.get_param('~can_interface', self.DEFAULT_CAN_INTERFACE)
        self.can_id = rospy.get_param('~can_id', self.DEFAULT_CAN_ID)
        self.pwm_min_old = rospy.get_param('~pwm_min_old', self.PWM_MIN_OLD)
        self.pwm_max_old = rospy.get_param('~pwm_max_old', self.PWM_MAX_OLD)
        self.pwm_min_new = rospy.get_param('~pwm_min_new', self.PWM_MIN_NEW)
        self.pwm_max_new = rospy.get_param('~pwm_max_new', self.PWM_MAX_NEW)
        self.rpm_base = rospy.get_param('~rpm_base', self.RPM_BASE)
        self.rpm_max = rospy.get_param('~rpm_max', self.RPM_MAX)

        # Initialize RPM value
        self.scaledrpm_value = 0

        # Set up the CAN interface with error handling
        try:
            self.bus = can.interface.Bus(channel=self.can_interface, interface='socketcan')
            rospy.loginfo(f'Initialized CAN bus on {self.can_interface}')
        except Exception as e:
            rospy.logerr(f'Failed to initialize CAN bus on {self.can_interface}: {e}')
            rospy.signal_shutdown('CAN bus initialization failed')

        # Subscribe to the /throttle_pwm topic
        rospy.Subscriber("/motor_rpm", Float32, self.throttle_callback)

        # Register shutdown hook for cleanup
        rospy.on_shutdown(self.shutdown_hook)

        # Set up a timer to publish CAN messages at 100 Hz
        self.publish_timer = rospy.Timer(rospy.Duration(0.01), self.publish_can_message)  # 10ms

    def scale_value(self, value):
        """
        Scales the input PWM value from the old range to the new range.
        """
        try:
            value_int = int(value)
        except ValueError:
            rospy.logwarn(f'Invalid PWM value type: {value}')
            return 0

        if value_int < self.pwm_min_old:
            rospy.logdebug(f'PWM value {value_int} below min_old {self.pwm_min_old}. Scaling to {self.pwm_min_new}.')
            return self.pwm_min_new
        elif value_int > self.pwm_max_old:
            rospy.logwarn(f'PWM value {value_int} exceeds max_old {self.pwm_max_old}. Clipping to {self.pwm_max_new}.')
            return self.pwm_max_new
        else:
            scaled = ((value_int - self.pwm_min_old) / (self.pwm_max_old - self.pwm_min_old)) * (self.pwm_max_new - self.pwm_min_new) + self.pwm_min_new
            rospy.logdebug(f'Scaled PWM value: {scaled}')
            return scaled

    def throttle_callback(self, msg):
        """
        Callback function for the /throttle_pwm topic.
        """
        if msg is None:
            rospy.logwarn('Received empty throttle PWM message.')
            return

        # scaled_rpm = self.scale_value(msg.data)
        scaled_rpm = msg.data
        rospy.loginfo(scaled_rpm)
        if scaled_rpm is None:
            rospy.logerr("I am stopping")
            # rospy.signal_shutdown()
            import sys
            sys.exit()
        self.scaledrpm_value = int(scaled_rpm)
        rospy.loginfo(f'Received and scaled RPM value: {self.scaledrpm_value}')

    def publish_can_message(self, event):
        """
        Publishes the scaled RPM value as a CAN message.
        """
        
        if self.scaledrpm_value != 0:

            adjusted_rpm = math.floor(self.scaledrpm_value) + self.rpm_base

            # Clip to 16-bit range
            self.rpm_value = min(max(adjusted_rpm, 0), self.rpm_max)
            hex_value = f'{self.rpm_value:04X}'
            high_byte = int(hex_value[:2], 16)
            low_byte = int(hex_value[2:], 16)
            
            # Prepare the CAN message
            data = [0x00, 0x7D, low_byte, high_byte, 0xF0, 0x00, 0x00, 0x00]
            msg = can.Message(arbitration_id=self.can_id, data=data, is_extended_id=True)

            try:
                self.bus.send(msg)
                rospy.loginfo(f'CAN message sent: {msg}')
                pass
            except can.CanError as e:
                rospy.logerr(f'Error sending CAN message: {e}')
        
        else:
            data = [0x00, 0x7D, 0x00, 0x7D, 0xF0, 0x00, 0x00, 0x00]
            msg = can.Message(arbitration_id=self.can_id, data=data)
            rospy.loginfo('Scaled RPM value is zero; sending 0 rpm CAN message.')

    def shutdown_hook(self):
        """
        Cleanup function to be called on node shutdown.
        """
        try:
            self.bus.shutdown()
            rospy.loginfo('CAN bus shutdown successfully.')
        except AttributeError:
            rospy.logwarn('CAN bus was not initialized; nothing to shutdown.')
        except Exception as e:
            rospy.logerr(f'Error during CAN bus shutdown: {e}')

if __name__ == '__main__':
    try:
        controller = MotorController()
        rospy.spin()  # Keeps the node from exiting until ROS shuts down
    except rospy.ROSInterruptException:
        pass
