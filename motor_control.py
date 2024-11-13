import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32 
import can
import time
import math

class MotorController:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('motor_controller', anonymous=True)

        rospy.Subscriber("/motor_rpm", Float32, self.throttle_callback)

        # Subscribe to the cmd_vel topic
        # rospy.Subscriber('cmd_vel', Twist, self.twist_callback)
        
        # Set up the CAN interface
        self.can_interface = 'can0'
        self.bus = can.interface.Bus(channel=self.can_interface, interface='socketcan')
        self.scaledrpm_value = 0
        # Initialize rpm_value
        self.rpm_value = 32000
        
        # Set up a rate of 10ms for publishing CAN messages
        self.rate = rospy.Rate(100)  # 100 Hz (10ms)
        
        # Main loop
        while not rospy.is_shutdown():
            self.publish_can_message()
            self.rate.sleep()
    
    def scale_value(self, value, min_old=950, max_old=2000, min_new=0, max_new=1000):
        # print("rpm",type(value))
        if int(value)<950:
            return 0
        value = int(value)
        return ((value - min_old) / (max_old - min_old)) * (max_new - min_new) + min_new

    def throttle_callback(self, msg):
        # # Scale linear.x to motor RPM
        # print(111111111111111111111111111111111111111)
        # linear_x = msg.linear.x
        # print(linear_x)
        # scaled_rpm = self.scale_linear_x_to_rpm(linear_x)
        # Add 32000 to the RPM value
        scaled_rpm=msg.data
        # print(type(int(msg.data)))
        self.scaledrpm_value = int(msg.data)
        
 
    # def scale_linear_x_to_rpm(self, linear_x):
    #     # Clip the linear.x values between -5 to 5
    #     linear_x_clipped = max(min(linear_x, 5), -5)
        
    #     # Scale to RPM range -2000 to 2000
    #     return int((linear_x_clipped / 5.0) * 2000)
 
    def publish_can_message(self):
        # Convert the RPM value to a hex string and split into high and low bytes
        if self.scaledrpm_value!=0:
            # rescaled_rpm = self.scale_value(self.scaledrpm_value)
            rescaled_rpm = self.scaledrpm_value
            if rescaled_rpm>500:
                print(rescaled_rpm)
            adjusted_rpm = math.floor(rescaled_rpm) + 32000
        
            # Convert to a 16-bit value and clip to valid range
            self.rpm_value = min(max(adjusted_rpm, 0), 65535)
            hex_value = f'{self.rpm_value:04X}'
            high_byte = int(hex_value[:2], 16)
            low_byte = int(hex_value[2:], 16)
            
            # Prepare the CAN message with the replaced low and high bytes
            can_id = 0x0C07BDA7
            data = [0x00, 0x7D, low_byte, high_byte, 0xF0, 0x00, 0x00, 0x00]
            # print("hellooo", data)
            msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=True)
            
            try:
                # self.bus.send(msg)
                rospy.loginfo(f'{self.scaledrpm_value}  CAN message sent: {msg}')
            except can.CanError as e:
                rospy.logerr(f'Error sending CAN message: {e}')
 
if __name__ == '__main__':
    try:
        MotorController()
    except rospy.ROSInterruptException:
        pass