# Header for the message
std_msgs/Header header

# Error Code 1 Low Byte
bool over_voltage_severely
bool over_voltage_reserve
bool over_heat_severely
bool over_heat_reserve
bool over_current
bool igbt_fault
bool main_contactor_fault
bool pre_charge_fault

# Error Code 1 High Byte
bool can_communication_fault
bool auxiliary_12v_fault
bool save_1
bool overspeed
bool locked_rotor_severely
bool save_2
bool under_voltage_severely
bool under_voltage_reserve

# Error Code 2 Low Byte
bool save_3
bool save_4
bool save_5
bool trend_to_zero
bool save_6
bool rotary_transformer_hardware_fault
bool save_7
bool neutral_point_voltage_fault

# Error Code 2 High Byte
bool motor_over_heat_severely
bool motor_over_heat_reserve
bool auto_malfunction_inspection
bool save_8
bool save_9
bool save_10
bool save_11
bool save_12
