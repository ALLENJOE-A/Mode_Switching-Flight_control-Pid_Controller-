################################################################################
# Cadence Genus Synthesis Script: ps02_top
# Project: Silicon Dawn VLSI Hackathon 2026 (PS-02 Mode-Switching PID Controller)
# Authors: Allen Joe A, T Venkata Sai Neeraj
# Usage: genus -f scripts/synthesis/genus_synth.tcl
################################################################################

set DESIGN "ps02_top"
set SCRIPT_DIR [file normalize [file dirname [info script]]]
set PROJ_DIR   [file normalize "${SCRIPT_DIR}/../.."]

# Project Directory Structure
set RTL_DIR "${PROJ_DIR}/rtl"
set SDC_FILE "${PROJ_DIR}/constraints/ps02_top.sdc"
set REP_DIR "${PROJ_DIR}/results/synthesis/reports"
set OUT_DIR "${PROJ_DIR}/results/synthesis/outputs"

# Ensure output directories exist
file mkdir ${REP_DIR}
file mkdir ${OUT_DIR}

puts "================================================================================"
puts "  Initializing Cadence Genus Synthesis for ${DESIGN}"
puts "  Project Root : ${PROJ_DIR}"
puts "  RTL Source   : ${RTL_DIR}"
puts "  SDC File     : ${SDC_FILE}"
puts "================================================================================"

# ------------------------------------------------------------------------------
# 1. Technology Library Configuration
# ------------------------------------------------------------------------------
# Supports:
#   A) Environment variable TARGET_LIBRARY
#   B) User-defined scripts/synthesis/genus_config.tcl
# ------------------------------------------------------------------------------
set target_lib_found 0

if {[info exists env(TARGET_LIBRARY)] && [file exists $env(TARGET_LIBRARY)]} {
    set_db target_library $env(TARGET_LIBRARY)
    set_db link_library   $env(TARGET_LIBRARY)
    set target_lib_found 1
    puts "  [CONFIG] Loaded target library from env(TARGET_LIBRARY): $env(TARGET_LIBRARY)"
} elseif {[file exists "${SCRIPT_DIR}/genus_config.tcl"]} {
    source "${SCRIPT_DIR}/genus_config.tcl"
    set target_lib_found 1
    puts "  [CONFIG] Loaded configuration from ${SCRIPT_DIR}/genus_config.tcl"
} else {
    puts "--------------------------------------------------------------------------------"
    puts "  [PREREQUISITE WARNING] Target standard-cell library not specified."
    puts "  To map to technology gates, specify a target Liberty (.lib) library:"
    puts "    1. Set the environment variable: export TARGET_LIBRARY=/path/to/slow.lib"
    puts "    2. Or copy 'genus_config.tcl.template' to 'genus_config.tcl' and set paths."
    puts "  Proceeding with GTECH generic elaboration only."
    puts "--------------------------------------------------------------------------------"
}

# ------------------------------------------------------------------------------
# 2. Read Synthesizable Verilog-2001 Sources in Dependency Order
# ------------------------------------------------------------------------------
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

# ------------------------------------------------------------------------------
# 3. Elaborate and Bind Hierarchy
# ------------------------------------------------------------------------------
elaborate ${DESIGN}
check_design -unresolved

# ------------------------------------------------------------------------------
# 4. Apply 50 MHz Timing Constraints
# ------------------------------------------------------------------------------
read_sdc ${SDC_FILE}

# ------------------------------------------------------------------------------
# 5. Generic Synthesis & Technology Mapping
# ------------------------------------------------------------------------------
set_db / .syn_generic_effort high
syn_generic

if {$target_lib_found} {
    set_db / .syn_map_effort high
    syn_map

    set_db / .syn_opt_effort high
    syn_opt

    # --------------------------------------------------------------------------
    # 6. Generate Verification & QoR Reports
    # --------------------------------------------------------------------------
    report_timing -nworst 10 > ${REP_DIR}/${DESIGN}_timing.rpt
    report_area             > ${REP_DIR}/${DESIGN}_area.rpt
    report_gates            > ${REP_DIR}/${DESIGN}_gates.rpt
    report_power            > ${REP_DIR}/${DESIGN}_power.rpt
    report_qor              > ${REP_DIR}/${DESIGN}_qor.rpt

    # --------------------------------------------------------------------------
    # 7. Export Gate-Level Netlist and Constraints for Innovus PnR
    # --------------------------------------------------------------------------
    write_hdl -mapped > ${OUT_DIR}/${DESIGN}_synth.v
    write_sdc         > ${OUT_DIR}/${DESIGN}_synth.sdc
    puts "================================================================================"
    puts "  Technology Mapping Complete! Netlist exported to:"
    puts "    ${OUT_DIR}/${DESIGN}_synth.v"
    puts "================================================================================"
} else {
    # Export generic elaborated netlist
    write_hdl > ${OUT_DIR}/${DESIGN}_elaborated.v
    puts "================================================================================"
    puts "  Elaboration Complete! Generic HDL exported to:"
    puts "    ${OUT_DIR}/${DESIGN}_elaborated.v"
    puts "================================================================================"
}
