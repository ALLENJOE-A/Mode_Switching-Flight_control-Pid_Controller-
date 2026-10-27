################################################################################
# Cadence Genus Synthesis Script: ps02_top
# Project: Silicon Dawn VLSI Hackathon 2026 (PS-02 Mode-Switching PID Controller)
# Authors: Allen Joe A, T Venkata Sai Neeraj
# Usage: genus -f scripts/synthesis/genus_synth.tcl
################################################################################

set DESIGN "ps02_top"
set SCRIPT_DIR [file dirname [info script]]
set PROJ_DIR [file normalize "${SCRIPT_DIR}/../.."]

# Directory setup
set RTL_DIR "${PROJ_DIR}/rtl"
set SDC_FILE "${PROJ_DIR}/constraints/ps02_top.sdc"
set REP_DIR "${PROJ_DIR}/results/synthesis/reports"
set OUT_DIR "${PROJ_DIR}/results/synthesis/outputs"

file mkdir ${REP_DIR}
file mkdir ${OUT_DIR}

# 1. Setup Library Paths (Configure according to local PDK installation)
# set_db init_lib_search_path { /path/to/target/pdk/lib }
# set_db target_library { slow.lib }
# set_db link_library   { slow.lib }

# 2. Read Synthesizable Verilog-2001 Sources
set_db init_hdl_search_path ${RTL_DIR}
read_hdl -v2001 [list \
    ${RTL_DIR}/fixed_point_saturation.v \
    ${RTL_DIR}/anti_windup.v \
    ${RTL_DIR}/derivative_filter.v \
    ${RTL_DIR}/pid_controller.v \
    ${RTL_DIR}/bumpless_transfer.v \
    ${RTL_DIR}/gain_bank.v \
    ${RTL_DIR}/config_registers.v \
    ${RTL_DIR}/mode_controller.v \
    ${RTL_DIR}/control_tick_generator.v \
    ${RTL_DIR}/watchdog_timer.v \
    ${RTL_DIR}/failsafe_controller.v \
    ${RTL_DIR}/attitude_controller.v \
    ${RTL_DIR}/rate_controller.v \
    ${RTL_DIR}/actuator_interface.v \
    ${RTL_DIR}/ps02_top.v \
]

# 3. Elaborate and Bind Hierarchy
elaborate ${DESIGN}
check_design -unresolved

# 4. Apply Timing Constraints
read_sdc ${SDC_FILE}

# 5. Synthesize to Generic Logic, Technology Map, and Optimize
set_db / .syn_generic_effort high
syn_generic

set_db / .syn_map_effort high
syn_map

set_db / .syn_opt_effort high
syn_opt

# 6. Generate Verification & QoR Reports
report_timing -nworst 10 > ${REP_DIR}/${DESIGN}_timing.rpt
report_area > ${REP_DIR}/${DESIGN}_area.rpt
report_gates > ${REP_DIR}/${DESIGN}_gates.rpt
report_power > ${REP_DIR}/${DESIGN}_power.rpt
report_qor > ${REP_DIR}/${DESIGN}_qor.rpt

# 7. Export Gate-Level Netlist and Constraints for Innovus PnR
write_hdl -mapped > ${OUT_DIR}/${DESIGN}_synth.v
write_sdc > ${OUT_DIR}/${DESIGN}_synth.sdc

puts "================================================================================"
puts "  Genus Synthesis Flow Completed Successfully for ${DESIGN}"
puts "================================================================================"
