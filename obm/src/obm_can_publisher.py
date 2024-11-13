#!/usr/bin/env python3

import rospy
import can
from can_msg.msg import controller, error, max_values, motor_stats, operation_state
from obm_can import CANMessageParser  
from std_msgs.msg import Header

class CANMessagePublisher:
    def __init__(self):
        rospy.init_node('can_message_publisher', anonymous=True)
        self.error_pub = rospy.Publisher('error_topic', error, queue_size=10)
        self.controller_pub = rospy.Publisher('controller_topic', controller, queue_size=10)
        self.max_values_pub = rospy.Publisher('max_values_topic', max_values, queue_size=10)
        self.motor_stats_pub = rospy.Publisher('motor_stats_topic', motor_stats, queue_size=10)
        self.operation_state_pub = rospy.Publisher('operation_state_topic', operation_state, queue_size=10)
        self.parser = CANMessageParser()

        # Set up the CAN interface (can0)
        self.can_interface = 'can0'
        self.bus = can.interface.Bus(channel=self.can_interface, bustype='socketcan')
        rospy.loginfo(f"Listening on CAN interface: {self.can_interface}")

    def publish_messages(self, can_id, data_str):
        parsed_message = self.parser.parse(can_id, data_str)
        table = parsed_message.get('Table')
        print(parsed_message)
        
        if table == 3:
            self.create_table_3_message(parsed_message)
        elif table == 2:
            self.create_table_2_message(parsed_message)
        elif table == 1:
            self.create_table_1_message(parsed_message)
        else:
            rospy.logwarn(f"Not A Suitable CAN_ID: {can_id}")

    def receive_can_message(self):
        try:
            while not rospy.is_shutdown():
                message = self.bus.recv()  # Blocking, waits for a CAN message
                if message:
                    can_id = hex(message.arbitration_id)  # Extract CAN ID
                    data_str = ''.join(format(x, '02X') for x in message.data)  # Convert data to hex string
                    print(can_id,data_str)
                    self.publish_messages(can_id, data_str)
        except can.CanError as e:
            rospy.logerr(f"CAN error: {e}")
        except KeyboardInterrupt:
            rospy.loginfo("Stopped listening to CAN messages.")
        finally:
            self.bus.shutdown()

    def create_table_1_message(self, parsed_data):
        message = controller()  
        message.header = Header()
        message.header.stamp = rospy.Time.now()
        message.front_end_voltage = parsed_data['Front End Voltage of Main Contactor']
        message.back_end_voltage = parsed_data['Back End Voltage of Main Contactor']
        message.ac_current_effective_value = parsed_data['AC Current Effective Value']
        message.motor_controller_temperature = parsed_data['Temperature of Motor Controller']
        message.motor_temperature = parsed_data['Temperature of Motor']
        self.controller_pub.publish(message)
    
    def create_table_2_message(self, parsed_message):
        performance_msg = motor_stats()
        performance_msg.header = Header()
        performance_msg.header.stamp = rospy.Time.now()
        performance_msg.motor_torque = parsed_message['Torque of Motor']
        performance_msg.motor_speed = parsed_message['Speed of Motor']
        performance_msg.dc_value = parsed_message['DC']


        status_msg = operation_state()
        status_msg.header = Header()
        status_msg.header.stamp = rospy.Time.now()
        status_msg.start_the_motor = parsed_message['Motor Controller Operation State Low Byte code']['START THE MOTOR']
        status_msg.mode = parsed_message['Motor Controller Operation State Low Byte code']['MODE']
        status_msg.ready = parsed_message['Motor Controller Operation State Low Byte code']['READY']
        status_msg.fault = parsed_message['Motor Controller Operation State Low Byte code']['FAULT']
        status_msg.drive_mode = parsed_message['Motor Controller Operation State Low Byte code']['1 MEANS DRIVE (0 MEANS BRAKE)']
        status_msg.forward_mode = parsed_message['Motor Controller Operation State Low Byte code']['1 MEANS FORWARD (0 MEANS REVERSE)']
        status_msg.standby = parsed_message['Motor Controller Operation State Low Byte code']['STANDBY']
        status_msg.start_stop = parsed_message['Motor Controller Operation State Low Byte code']['1 START (0 STOP)']
        status_msg.can_life_1 = parsed_message['Motor Controller Operation State High Byte code']['CAN LIFE 1']
        status_msg.can_life_2 = parsed_message['Motor Controller Operation State High Byte code']['CAN LIFE 2']
        status_msg.can_life_3 = parsed_message['Motor Controller Operation State High Byte code']['CAN LIFE 3']
        status_msg.can_life_4 = parsed_message['Motor Controller Operation State High Byte code']['CAN LIFE 4']
        status_msg.gear_state_1 = parsed_message['Motor Controller Operation State High Byte code']['GEAR STATE 1']
        status_msg.gear_state_2 = parsed_message['Motor Controller Operation State High Byte code']['GEAR STATE 2']
        status_msg.main_contactor = parsed_message['Motor Controller Operation State High Byte code']['MAIN CONTACTOR']
        status_msg.precharge = parsed_message['Motor Controller Operation State High Byte code']['PRECHARGE']
        self.motor_stats_pub.publish(performance_msg)
        self.operation_state_pub.publish(status_msg)

    def create_table_3_message(self, parsed_message):
        error_msg = error()
        error_msg.header = Header()
        error_msg.header.stamp = rospy.Time.now()
        error_msg.over_voltage_severely = parsed_message['Error 1 Low']['Over voltage severely']
        error_msg.over_voltage_reserve = parsed_message['Error 1 Low']['Over voltage reserve']
        error_msg.over_heat_severely = parsed_message['Error 1 Low']['Over heat severely (80 deg.C)']
        error_msg.over_heat_reserve = parsed_message['Error 1 Low']['Over heat (reserve)']
        error_msg.over_current = parsed_message['Error 1 Low']['Over current']
        error_msg.igbt_fault = parsed_message['Error 1 Low']['IGBT']
        error_msg.main_contactor_fault = parsed_message['Error 1 Low']['Main contactor']
        error_msg.pre_charge_fault = parsed_message['Error 1 Low']['Pre-charge']
        error_msg.can_communication_fault = parsed_message['Error 1 High']['CAN Communication fault']
        error_msg.auxiliary_12v_fault = parsed_message['Error 1 High']['12V auxiliary fault']
        error_msg.save_1 = parsed_message['Error 1 High']['Save_1']
        error_msg.overspeed = parsed_message['Error 1 High']['Overspeed']
        error_msg.locked_rotor_severely = parsed_message['Error 1 High']['Locked rotor severely']
        error_msg.save_2 = parsed_message['Error 1 High']['Save_2']
        error_msg.under_voltage_severely = parsed_message['Error 1 High']['Under voltage severely']
        error_msg.under_voltage_reserve = parsed_message['Error 1 High']['Under voltage reserve']
        max_values_msg = max_values()
        max_values_msg.header = Header()
        max_values_msg.header.stamp = rospy.Time.now()
        max_values_msg.max_torque = parsed_message['Max Torque']
        max_values_msg.max_speed = parsed_message['Max Speed']
        self.max_values_pub.publish(max_values_msg)
        self.error_pub.publish(error_msg)
    
if __name__ == '__main__':
    try:
        can_publisher = CANMessagePublisher()
        can_publisher.receive_can_message()  # Start receiving CAN messages
    except rospy.ROSInterruptException:
        pass
