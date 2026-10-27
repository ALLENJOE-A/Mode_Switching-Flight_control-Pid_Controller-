#!/usr/bin/env python3
"""
run_regression.py
Automated simulation regression runner for PS-02 Mode-Switching Hardware PID Controller.
Uses ModelSim (vlog/vsim) to compile Verilog-2001 sources, execute all self-checking
testbenches, verify pass/fail criteria, and generate execution logs.
"""

import os
import sys
import subprocess
import time
import shutil

# Configuration paths
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
RTL_DIR = os.path.join(PROJECT_ROOT, "rtl")
TB_DIR = os.path.join(PROJECT_ROOT, "tb")
SIM_DIR = os.path.join(PROJECT_ROOT, "sim")
LOG_DIR = os.path.join(SIM_DIR, "logs")
WORK_DIR = os.path.join(SIM_DIR, "work")

MODELSIM_BIN = r"C:\intelFPGA\18.1\modelsim_ase\win32aloem"
VLIB = os.path.join(MODELSIM_BIN, "vlib.exe")
VLOG = os.path.join(MODELSIM_BIN, "vlog.exe")
VSIM = os.path.join(MODELSIM_BIN, "vsim.exe")

# RTL Sources in dependency order
RTL_SOURCES = [
    os.path.join(RTL_DIR, "fixed_point_saturation.v"),
    os.path.join(RTL_DIR, "anti_windup.v"),
    os.path.join(RTL_DIR, "derivative_filter.v"),
    os.path.join(RTL_DIR, "pid_controller.v"),
    os.path.join(RTL_DIR, "bumpless_transfer.v"),
    os.path.join(RTL_DIR, "gain_bank.v"),
    os.path.join(RTL_DIR, "config_registers.v"),
    os.path.join(RTL_DIR, "mode_controller.v"),
    os.path.join(RTL_DIR, "control_tick_generator.v"),
    os.path.join(RTL_DIR, "watchdog_timer.v"),
    os.path.join(RTL_DIR, "failsafe_controller.v"),
    os.path.join(RTL_DIR, "attitude_controller.v"),
    os.path.join(RTL_DIR, "rate_controller.v"),
    os.path.join(RTL_DIR, "actuator_interface.v"),
    os.path.join(RTL_DIR, "ps02_top.v"),
]

# Testbenches to execute
TESTBENCHES = [
    ("tb_saturation", os.path.join(TB_DIR, "tb_saturation.v"), [os.path.join(RTL_DIR, "fixed_point_saturation.v")]),
    ("tb_derivative_filter", os.path.join(TB_DIR, "tb_derivative_filter.v"), [os.path.join(RTL_DIR, "fixed_point_saturation.v"), os.path.join(RTL_DIR, "derivative_filter.v")]),
    ("tb_anti_windup", os.path.join(TB_DIR, "tb_anti_windup.v"), [os.path.join(RTL_DIR, "anti_windup.v")]),
    ("tb_pid_controller", os.path.join(TB_DIR, "tb_pid_controller.v"), [os.path.join(RTL_DIR, "fixed_point_saturation.v"), os.path.join(RTL_DIR, "derivative_filter.v"), os.path.join(RTL_DIR, "anti_windup.v"), os.path.join(RTL_DIR, "pid_controller.v")]),
    ("tb_bumpless_transfer", os.path.join(TB_DIR, "tb_bumpless_transfer.v"), [os.path.join(RTL_DIR, "bumpless_transfer.v")]),
    ("tb_mode_controller", os.path.join(TB_DIR, "tb_mode_controller.v"), [os.path.join(RTL_DIR, "mode_controller.v")]),
    ("tb_watchdog_failsafe", os.path.join(TB_DIR, "tb_watchdog_failsafe.v"), [os.path.join(RTL_DIR, "watchdog_timer.v"), os.path.join(RTL_DIR, "failsafe_controller.v")]),
    ("tb_ps02_top", os.path.join(TB_DIR, "tb_ps02_top.v"), RTL_SOURCES),
]


def run_command(cmd, cwd=None):
    proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, cwd=cwd)
    return proc.returncode, proc.stdout


def main():
    print("=" * 80)
    print("   PS-02 HARDWARE PID CONTROLLER - VERILOG-2001 REGRESSION SUITE")
    print("=" * 80)

    os.makedirs(SIM_DIR, exist_ok=True)
    os.makedirs(LOG_DIR, exist_ok=True)

    # Initialize ModelSim work library
    if os.path.exists(WORK_DIR):
        try:
            shutil.rmtree(WORK_DIR)
        except Exception:
            pass

    print("[INFO] Creating ModelSim work library in sim/work...")
    code, out = run_command([VLIB, "work"], cwd=SIM_DIR)
    if code != 0:
        print(f"[ERROR] vlib failed:\n{out}")
        sys.exit(1)

    # Compile RTL files
    print("[INFO] Compiling synthesizable RTL modules with vlog...")
    vlog_cmd = [VLOG, "-work", "work"] + RTL_SOURCES
    code, out = run_command(vlog_cmd, cwd=SIM_DIR)
    if code != 0:
        print(f"[ERROR] RTL compilation failed:\n{out}")
        sys.exit(1)
    print("[PASS] All RTL modules compiled cleanly with zero errors.")

    results = []
    total_start = time.time()

    for tb_name, tb_file, req_rtl in TESTBENCHES:
        print("-" * 80)
        print(f"[RUNNING] Testbench: {tb_name}")
        tb_log = os.path.join(LOG_DIR, f"{tb_name}.log")

        # Compile testbench
        tb_compile_cmd = [VLOG, "-work", "work", tb_file]
        code, out = run_command(tb_compile_cmd, cwd=SIM_DIR)
        if code != 0:
            print(f"[FAIL] {tb_name} compilation failed:\n{out}")
            results.append((tb_name, "FAIL", "Compilation Error", 0.0))
            continue

        # Run simulation in batch mode
        t0 = time.time()
        vsim_cmd = [VSIM, "-c", "-do", "run -all; quit -f", f"work.{tb_name}"]
        code, out = run_command(vsim_cmd, cwd=SIM_DIR)
        elapsed = time.time() - t0

        # Save log file
        with open(tb_log, "w", encoding="utf-8") as f:
            f.write(out)

        # Check output for PASS/FAIL
        passed = ("*** TEST PASSED" in out or "ALL 20 VERIFICATION CHECKLIST ITEMS PASSED" in out) and ("[FAIL]" not in out) and ("[FATAL]" not in out)
        status = "PASS" if passed else "FAIL"

        print(f"[{status}] {tb_name} finished in {elapsed:.2f}s | Log: sim/logs/{tb_name}.log")
        results.append((tb_name, status, "OK" if passed else "Check Log", elapsed))

    total_time = time.time() - total_start
    print("=" * 80)
    print("   REGRESSION EXECUTION SUMMARY")
    print("=" * 80)
    print(f"{'Testbench Name':<28} | {'Status':<8} | {'Duration':<10} | {'Details'}")
    print("-" * 80)
    all_pass = True
    for name, stat, det, elap in results:
        if stat != "PASS":
            all_pass = False
        print(f"{name:<28} | {stat:<8} | {elap:<9.2f}s | {det}")
    print("-" * 80)
    print(f"Total Regression Duration: {total_time:.2f}s")
    if all_pass:
        print("[SUCCESS] ALL VERIFICATION TESTBENCHES PASSED CLEANLY (100% REGRESSION PASS RATE)")
    else:
        print("[FAILURE] REGRESSION HAD FAILING TESTS")
        sys.exit(1)


if __name__ == "__main__":
    main()
