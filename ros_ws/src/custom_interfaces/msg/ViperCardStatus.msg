std_msgs/Header header

# Input Current (milliamps)
float64 input_current

# Temperature
float64 temperature_backplane
float64 temperature_card_a
float64 temperature_card_b

# Output For A
float64 output_voltage_a
float64 output_current_a
float64 output_power_a

# Output For B (unused for high power cards)
float64 output_voltage_b
float64 output_current_b
float64 output_power_b

# Faults
bool input_fault false
bool output_a_fault false
bool output_b_fault false

#Diagnostic faults