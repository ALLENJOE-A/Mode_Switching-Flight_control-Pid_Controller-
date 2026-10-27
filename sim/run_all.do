# ModelSim Tcl Script for PS-02 Hardware PID Controller
# Usage in ModelSim GUI/CLI: do run_all.do

# Create work library
vlib work

# Compile synthesizable RTL
vlog -work work ../rtl/fixed_point_saturation.v
vlog -work work ../rtl/anti_windup.v
vlog -work work ../rtl/derivative_filter.v
vlog -work work ../rtl/pid_controller.v
vlog -work work ../rtl/bumpless_transfer.v
vlog -work work ../rtl/gain_bank.v
vlog -work work ../rtl/config_registers.v
vlog -work work ../rtl/mode_controller.v
vlog -work work ../rtl/control_tick_generator.v
vlog -work work ../rtl/watchdog_timer.v
vlog -work work ../rtl/failsafe_controller.v
vlog -work work ../rtl/attitude_controller.v
vlog -work work ../rtl/rate_controller.v
vlog -work work ../rtl/actuator_interface.v
vlog -work work ../rtl/ps02_top.v

# Compile Testbenches
vlog -work work ../tb/tb_saturation.v
vlog -work work ../tb/tb_derivative_filter.v
vlog -work work ../tb/tb_anti_windup.v
vlog -work work ../tb/tb_pid_controller.v
vlog -work work ../tb/tb_bumpless_transfer.v
vlog -work work ../tb/tb_mode_controller.v
vlog -work work ../tb/tb_watchdog_failsafe.v
vlog -work work ../tb/tb_ps02_top.v

# Run Testbenches
puts "\n=== Running tb_saturation ==="
vsim -c -do "run -all; quit -f" work.tb_saturation

puts "\n=== Running tb_derivative_filter ==="
vsim -c -do "run -all; quit -f" work.tb_derivative_filter

puts "\n=== Running tb_anti_windup ==="
vsim -c -do "run -all; quit -f" work.tb_anti_windup

puts "\n=== Running tb_pid_controller ==="
vsim -c -do "run -all; quit -f" work.tb_pid_controller

puts "\n=== Running tb_bumpless_transfer ==="
vsim -c -do "run -all; quit -f" work.tb_bumpless_transfer

puts "\n=== Running tb_mode_controller ==="
vsim -c -do "run -all; quit -f" work.tb_mode_controller

puts "\n=== Running tb_watchdog_failsafe ==="
vsim -c -do "run -all; quit -f" work.tb_watchdog_failsafe

puts "\n=== Running tb_ps02_top ==="
vsim -c -do "run -all; quit -f" work.tb_ps02_top
