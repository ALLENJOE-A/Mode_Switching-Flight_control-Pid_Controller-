################################################################################
# SDC Timing Constraints: ps02_top (Mode-Switching PID Flight Controller)
# Project: Silicon Dawn VLSI Hackathon 2026 (PS-02)
# Authors: Allen Joe A, T Venkata Sai Neeraj
# Target Operating Frequency: 50 MHz (Period: 20.0 ns)
################################################################################

# Set timing units
set_units -time ns -resistance kOhm -capacitance pF -voltage V -current mA

################################################################################
# Clock Definitions
################################################################################
# Primary 50 MHz system clock with 50% nominal duty cycle
create_clock -name clk -period 20.000 -waveform {0.000 10.000} [get_ports clk]

# Clock uncertainty (jitter + margin)
set_clock_uncertainty -setup 0.200 [get_clocks clk]
set_clock_uncertainty -hold  0.100 [get_clocks clk]

# Clock transition / slew rate
set_clock_transition 0.150 [get_clocks clk]

################################################################################
# Input Delays (Referenced to clk)
# Accounting for PCB trace delay, external IMU ADC setup, and register setup
################################################################################
# Sensor telemetry bus (synchronous arrival with sensor_valid strobe)
set_input_delay -clock clk -max 3.500 [get_ports sensor_valid]
set_input_delay -clock clk -min 0.500 [get_ports sensor_valid]

set_input_delay -clock clk -max 3.500 [get_ports roll_angle_meas*]
set_input_delay -clock clk -min 0.500 [get_ports roll_angle_meas*]
set_input_delay -clock clk -max 3.500 [get_ports pitch_angle_meas*]
set_input_delay -clock clk -min 0.500 [get_ports pitch_angle_meas*]
set_input_delay -clock clk -max 3.500 [get_ports yaw_angle_meas*]
set_input_delay -clock clk -min 0.500 [get_ports yaw_angle_meas*]
set_input_delay -clock clk -max 3.500 [get_ports roll_rate_meas*]
set_input_delay -clock clk -min 0.500 [get_ports roll_rate_meas*]
set_input_delay -clock clk -max 3.500 [get_ports pitch_rate_meas*]
set_input_delay -clock clk -min 0.500 [get_ports pitch_rate_meas*]
set_input_delay -clock clk -max 3.500 [get_ports yaw_rate_meas*]
set_input_delay -clock clk -min 0.500 [get_ports yaw_rate_meas*]
set_input_delay -clock clk -max 3.500 [get_ports altitude_meas*]
set_input_delay -clock clk -min 0.500 [get_ports altitude_meas*]
set_input_delay -clock clk -max 3.500 [get_ports vert_vel_meas*]
set_input_delay -clock clk -min 0.500 [get_ports vert_vel_meas*]

# Pilot setpoints
set_input_delay -clock clk -max 3.500 [get_ports roll_sp*]
set_input_delay -clock clk -min 0.500 [get_ports roll_sp*]
set_input_delay -clock clk -max 3.500 [get_ports pitch_sp*]
set_input_delay -clock clk -min 0.500 [get_ports pitch_sp*]
set_input_delay -clock clk -max 3.500 [get_ports yaw_sp*]
set_input_delay -clock clk -min 0.500 [get_ports yaw_sp*]
set_input_delay -clock clk -max 3.500 [get_ports altitude_sp*]
set_input_delay -clock clk -min 0.500 [get_ports altitude_sp*]
set_input_delay -clock clk -max 3.500 [get_ports throttle_base*]
set_input_delay -clock clk -min 0.500 [get_ports throttle_base*]

# Mode selection controls
set_input_delay -clock clk -max 3.000 [get_ports mode_cmd*]
set_input_delay -clock clk -min 0.500 [get_ports mode_cmd*]
set_input_delay -clock clk -max 3.000 [get_ports mode_cmd_valid]
set_input_delay -clock clk -min 0.500 [get_ports mode_cmd_valid]

# Register access bus
set_input_delay -clock clk -max 3.000 [get_ports reg_wr_en]
set_input_delay -clock clk -min 0.500 [get_ports reg_wr_en]
set_input_delay -clock clk -max 3.000 [get_ports reg_rd_en]
set_input_delay -clock clk -min 0.500 [get_ports reg_rd_en]
set_input_delay -clock clk -max 3.000 [get_ports reg_addr*]
set_input_delay -clock clk -min 0.500 [get_ports reg_addr*]
set_input_delay -clock clk -max 3.000 [get_ports reg_wr_data*]
set_input_delay -clock clk -min 0.500 [get_ports reg_wr_data*]

# Synchronous reset
set_input_delay -clock clk -max 3.000 [get_ports rst]
set_input_delay -clock clk -min 0.500 [get_ports rst]

################################################################################
# Output Delays (Referenced to clk)
################################################################################
# Motor command PWM generator feeds
set_output_delay -clock clk -max 3.000 [get_ports motor0_cmd*]
set_output_delay -clock clk -min 0.500 [get_ports motor0_cmd*]
set_output_delay -clock clk -max 3.000 [get_ports motor1_cmd*]
set_output_delay -clock clk -min 0.500 [get_ports motor1_cmd*]
set_output_delay -clock clk -max 3.000 [get_ports motor2_cmd*]
set_output_delay -clock clk -min 0.500 [get_ports motor2_cmd*]
set_output_delay -clock clk -max 3.000 [get_ports motor3_cmd*]
set_output_delay -clock clk -min 0.500 [get_ports motor3_cmd*]
set_output_delay -clock clk -max 3.000 [get_ports motor_valid]
set_output_delay -clock clk -min 0.500 [get_ports motor_valid]

# Register read interface
set_output_delay -clock clk -max 3.000 [get_ports reg_rd_data*]
set_output_delay -clock clk -min 0.500 [get_ports reg_rd_data*]
set_output_delay -clock clk -max 3.000 [get_ports reg_ack]
set_output_delay -clock clk -min 0.500 [get_ports reg_ack]

# Status and telemetry flags
set_output_delay -clock clk -max 3.000 [get_ports current_mode*]
set_output_delay -clock clk -min 0.500 [get_ports current_mode*]
set_output_delay -clock clk -max 3.000 [get_ports failsafe_active]
set_output_delay -clock clk -min 0.500 [get_ports failsafe_active]
set_output_delay -clock clk -max 3.000 [get_ports watchdog_alarm]
set_output_delay -clock clk -min 0.500 [get_ports watchdog_alarm]
set_output_delay -clock clk -max 3.000 [get_ports saturation_flags*]
set_output_delay -clock clk -min 0.500 [get_ports saturation_flags*]
set_output_delay -clock clk -max 3.000 [get_ports bumpless_active]
set_output_delay -clock clk -min 0.500 [get_ports bumpless_active]

################################################################################
# Design Rules & Driving Cells
################################################################################
set_max_fanout 16 [current_design]
set_max_transition 0.500 [current_design]
