#!/bin/bash
# ============================================================================
# run_nclaunch.sh
# Automated simulation regression script for Cadence NCLaunch / Incisive
# Project: PS-02 Hardware PID Controller
# ============================================================================

# Ensure we exit on any failure
set -e

echo "================================================================================"
echo "   PS-02 HARDWARE PID CONTROLLER - CADENCE NC-VERILOG REGRESSION SUITE"
echo "================================================================================"

# Create work directory
rm -rf INCA_libs work
mkdir -p work

# List of testbenches to run
declare -a TESTBENCHES=(
    "tb_saturation"
    "tb_derivative_filter"
    "tb_anti_windup"
    "tb_pid_controller"
    "tb_bumpless_transfer"
    "tb_mode_controller"
    "tb_watchdog_failsafe"
    "tb_ps02_top"
)

# Step 1: Compile all RTL and Testbench files
echo "[INFO] Compiling Verilog-2001 sources..."
ncvlog -v2001 -work work -f nc_files.f -update -messages

mkdir -p logs

PASS_COUNT=0
FAIL_COUNT=0

# Step 2 & 3: Elaborate and Simulate each testbench
echo "[INFO] Running Regressions..."

for TB in "${TESTBENCHES[@]}"
do
    echo "--------------------------------------------------------------------------------"
    echo "[RUNNING] Testbench: $TB"
    
    # Elaborate
    ncelab -work work -update -messages -access +rwc work.$TB
    
    # Simulate
    # -run runs the simulation until $finish, -exit closes ncsim afterwards
    ncsim -work work -messages -run -exit work.$TB > logs/$TB.log 2>&1
    
    # Check PASS/FAIL in the log
    if grep -q "\*\*\* TEST PASSED" logs/$TB.log || grep -q "\*\*\* ALL VERIFICATION CHECKLIST ITEMS PASSED SUCCESSFULLY!" logs/$TB.log; then
        if grep -q "\[FAIL\]" logs/$TB.log || grep -q "\[FATAL\]" logs/$TB.log; then
            echo "[FAIL] $TB failed. See logs/$TB.log"
            FAIL_COUNT=$((FAIL_COUNT+1))
        else
            echo "[PASS] $TB passed."
            PASS_COUNT=$((PASS_COUNT+1))
        fi
    else
        echo "[FAIL] $TB failed or did not finish cleanly. See logs/$TB.log"
        FAIL_COUNT=$((FAIL_COUNT+1))
    fi
done

echo "================================================================================"
echo "   REGRESSION EXECUTION SUMMARY: $PASS_COUNT PASSED, $FAIL_COUNT FAILED"
echo "================================================================================"

if [ $FAIL_COUNT -eq 0 ]; then
    echo "[SUCCESS] ALL VERIFICATION TESTBENCHES PASSED CLEANLY (100% REGRESSION PASS RATE)"
    exit 0
else
    echo "[FAILURE] REGRESSION HAD FAILING TESTS"
    exit 1
fi
