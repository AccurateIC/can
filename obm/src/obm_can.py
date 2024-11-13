#!/usr/bin/env python3
class CANMessageParser:

    def parse_hex_value(self,data, high_byte_index, low_byte_index, subtract_value=32000):
        hex_value = data[high_byte_index] + data[low_byte_index]
        return int(hex_value, 16) - subtract_value

    def parse_bin_value(self,data, index):
        return bin(int(data[index], 16))[2:].zfill(8)

    def interpret_motor_controller_operation_state_low(self,binary_state):
        return {
            "START THE MOTOR": binary_state[0] == '1',
            "MODE": binary_state[1] == '1',
            "READY": binary_state[2] == '1',
            "FAULT": binary_state[3] == '1',
            "1 MEANS DRIVE (0 MEANS BRAKE)": binary_state[4] == '1',
            "1 MEANS FORWARD (0 MEANS REVERSE)": binary_state[5] == '1',
            "STANDBY": binary_state[6] == '1',
            "1 START (0 STOP)": binary_state[7] == '1',
        }

    def interpret_motor_controller_operation_state_high(self,binary_state):
        return {
            "CAN LIFE 1": binary_state[0] == '1',
            "CAN LIFE 2": binary_state[1] == '1',
            "CAN LIFE 3": binary_state[2] == '1',
            "CAN LIFE 4": binary_state[3] == '1',
            "GEAR STATE 1": binary_state[4] == '1',
            "GEAR STATE 2": binary_state[5] == '1',
            "MAIN CONTACTOR": binary_state[6] == '1',
            "PRECHARGE": binary_state[7] == '1',
        }

    def interpret_error_code_1_low(self,binary_error):
        return {
            "Over voltage severely": binary_error[0] == '1',
            "Over voltage reserve": binary_error[1] == '1',
            "Over heat severely (80 deg.C)": binary_error[2] == '1',
            "Over heat (reserve)": binary_error[3] == '1',
            "Over current": binary_error[4] == '1',
            "IGBT": binary_error[5] == '1',
            "Main contactor": binary_error[6] == '1',
            "Pre-charge": binary_error[7] == '1',
        }

    def interpret_error_code_1_high(self,binary_error):
        return {
            "CAN Communication fault": binary_error[0] == '1',
            "12V auxiliary fault": binary_error[1] == '1',
            "Save_1": binary_error[2] == '1',
            "Overspeed": binary_error[3] == '1',
            "Locked rotor severely": binary_error[4] == '1',
            "Save_2": binary_error[5] == '1',
            "Under voltage severely": binary_error[6] == '1',
            "Under voltage reserve": binary_error[7] == '1',
        }

    def interpret_error_code_2_low(self,binary_error):
        return {
            "Save_1": binary_error[0] == '1',
            "Save_2": binary_error[1] == '1',
            "Save_3": binary_error[2] == '1',
            "Trend to 0": binary_error[3] == '1',
            "Save_4": binary_error[4] == '1',
            "Hardware of rotary transformer": binary_error[5] == '1',
            "Save_5": binary_error[6] == '1',
            "Neutral point voltage": binary_error[7] == '1',
        }

    def interpret_error_code_2_high(self,binary_error):
        return {
            "Motor over heat severely": binary_error[0] == '1',
            "Motor over heat reserve": binary_error[1] == '1',
            "Auto malfunctioning inspection": binary_error[2] == '1',
            "Save_1": binary_error[3] == '1',
            "Save_2": binary_error[4] == '1',
            "Save_3": binary_error[5] == '1',
            "Save_4": binary_error[6] == '1',
            "Save_5": binary_error[7] == '1',
        }

    def get_active_modes(self,modes):
        return {key: value for key, value in modes.items() if value}

    def parse(self, can_id, data_str):
        data = [data_str[i:i+2] for i in range(0, len(data_str), 2)]
        parsed_values = {}

        if can_id == "0xcf001bd":
            parsed_values["Table"]=1
            parsed_values["Front End Voltage of Main Contactor"] = self.parse_hex_value(data, 1, 0)
            parsed_values["Back End Voltage of Main Contactor"] = self.parse_hex_value(data, 3, 2)
            parsed_values["AC Current Effective Value"] = self.parse_hex_value(data, 5, 4)
            parsed_values["Temperature of Motor Controller"] = int(data[6], 16) -  40
            parsed_values["Temperature of Motor"] = int(data[7], 16) - 40

        elif can_id == "0xcf000bd":
            parsed_values["Table"]=2
            parsed_values["Torque of Motor"] = self.parse_hex_value(data, 1, 0)
            parsed_values["Speed of Motor"] = self.parse_hex_value(data, 3, 2)
            parsed_values["DC"] = self.parse_hex_value(data, 5, 4)
            parsed_values["Motor Controller Operation State Low Byte"] = self.parse_bin_value(data, 6)
            modes = self.interpret_motor_controller_operation_state_low(parsed_values["Motor Controller Operation State Low Byte"])
            parsed_values["Motor Controller Operation State Low Byte code"] = modes
            parsed_values["Motor Controller Operation State High Byte"] = self.parse_bin_value(data, 7)
            modes_1 = self.interpret_motor_controller_operation_state_high(parsed_values["Motor Controller Operation State High Byte"])
            parsed_values["Motor Controller Operation State High Byte code"] = modes_1

        elif can_id == "0xcf002bd":
            parsed_values["Table"]=3
            parsed_values["Error Code 1 Low"] = self.parse_bin_value(data, 0)
            low_mode = self.interpret_error_code_1_low(parsed_values["Error Code 1 Low"])
            parsed_values["Error 1 Low"] = low_mode

            parsed_values["Error Code 1 High"] = self.parse_bin_value(data, 1)
            high_mode = self.interpret_error_code_1_high(parsed_values["Error Code 1 High"])
            parsed_values["Error 1 High"] = high_mode

            parsed_values["Error Code 2 Low"] = self.parse_bin_value(data, 2)
            low_mode_1 = self.interpret_error_code_2_low(parsed_values["Error Code 2 Low"])
            parsed_values["Error 2 Low"] = low_mode_1

            parsed_values["Error Code 2 High"] = self.parse_bin_value(data, 3)
            high_mode_1 = self.interpret_error_code_2_high(parsed_values["Error Code 2 High"])
            parsed_values["Error 2 High"] = high_mode_1

            parsed_values["Max Torque"] = self.parse_hex_value(data, 5, 4)
            parsed_values["Max Speed"] = self.parse_hex_value(data, 7, 6)

        return parsed_values


# parser = CANMessageParser()

# can_id = "0xCF002BD"
# data_str = "30800000007D6C9D"
# parsed_message = parser.parse(can_id, data_str)
# print(parsed_message)

# can_id = "0xCF000DB"
# data_str = "0A1B2C3D4E5F6070"
# parsed_message = parser.parse(can_id, data_str)
# print(parsed_message)
