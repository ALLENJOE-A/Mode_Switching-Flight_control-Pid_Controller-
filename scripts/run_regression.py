#!/usr/bin/env python3
"""
run_regression.py
Automated simulation regression runner for PS-02 Mode-Switching Hardware PID Controller.
Supports ModelSim / QuestaSim on Windows and Linux.
Features:
- Configurable tool paths via environment variables or PATH discovery
- Clear error handling distinguishing infrastructure failures from test assertion failures
- Deterministic log output in sim/logs/
- Single-test execution or full regression suite
"""

import os
import sys
import subprocess
import time
import shutil
import glob

# Derive project root strictly relative to this script's directory
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
RTL_DIR = os.path.join(PROJECT_ROOT, "rtl")
TB_DIR = os.path.join(PROJECT_ROOT, "tb")
SIM_DIR = os.path.join(PROJECT_ROOT, "sim")
LOG_DIR = os.path.join(SIM_DIR, "logs")
WORK_DIR = os.path.join(SIM_DIR, "work")

# RTL Sources in strict dependency compilation order
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

# Testbench configurations
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


def locate_executable(name):
    """Searches for an executable with or without .exe extension."""
    exe_name = name + ".exe" if sys.platform.startswith("win") else name
    # 1. Check environment variable hints
    for env_var in ["MODELSIM_BIN", "MODELSIM_DIR", "QUESTA_BIN", "QUESTA_DIR", "SIMULATOR_DIR"]:
        env_val = os.environ.get(env_var)
        if env_val:
            candidate = os.path.join(env_val, exe_name)
            if os.path.isfile(candidate):
                return candidate
            candidate_raw = os.path.join(env_val, name)
            if os.path.isfile(candidate_raw):
                return candidate_raw

    # 2. Check system PATH
    found = shutil.which(name) or shutil.which(exe_name)
    if found:
        return found

    # 3. Check common default install paths on Windows
    if sys.platform.startswith("win"):
        patterns = [
            r"C:\intelFPGA*\*\modelsim_ase\win32aloem",
            r"C:\intelFPGA*\*\modelsim_ae\win32aloem",
            r"C:\modeltech*",
            r"C:\questasim*",
        ]
        for pat in patterns:
            for d in sorted(glob.glob(pat), reverse=True):
                cand = os.path.join(d, exe_name)
                if os.path.isfile(cand):
                    return cand

    return None


def run_command(cmd, cwd=None):
    proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, cwd=cwd)
    return proc.returncode, proc.stdout


def main():
    print("=" * 80)
    print("   PS-02 HARDWARE PID CONTROLLER - VERILOG-2001 REGRESSION SUITE")
    print("=" * 80)

    # Tool discovery
    vlib_bin = locate_executable("vlib")
    vlog_bin = locate_executable("vlog")
    vsim_bin = locate_executable("vsim")

    if not (vlib_bin and vlog_bin and vsim_bin):
        print("[INFRASTRUCTURE ERROR] ModelSim / QuestaSim tools not found on PATH or via MODELSIM_DIR.")
        print("Searched executables: vlib, vlog, vsim")
        print("To resolve, either:")
        print("  1. Add the simulator bin folder to your system PATH.")
        print("  2. Set the MODELSIM_DIR environment variable to the simulator bin directory:")
        print("     e.g., (PowerShell) $env:MODELSIM_DIR='C:\\intelFPGA\\18.1\\modelsim_ase\\win32aloem'")
        print("     e.g., (Bash)       export MODELSIM_DIR=/path/to/modelsim/bin")
        sys.exit(2)

    print(f"[TOOL FOUND] vlib : {vlib_bin}")
    print(f"[TOOL FOUND] vlog : {vlog_bin}")
    print(f"[TOOL FOUND] vsim : {vsim_bin}")

    os.makedirs(SIM_DIR, exist_ok=True)
    os.makedirs(LOG_DIR, exist_ok=True)

    # Initialize ModelSim work library
    if os.path.exists(WORK_DIR):
        try:
            shutil.rmtree(WORK_DIR)
        except Exception:
            pass

    print("[INFO] Initializing work library in sim/work...")
    code, out = run_command([vlib_bin, "work"], cwd=SIM_DIR)
    if code != 0:
        print(f"[INFRASTRUCTURE ERROR] vlib failed to initialize work library:\n{out}")
        sys.exit(2)

    # Compile RTL files
    print("[INFO] Compiling synthesizable RTL modules...")
    vlog_cmd = [vlog_bin, "-work", "work"] + RTL_SOURCES
    code, out = run_command(vlog_cmd, cwd=SIM_DIR)
    if code != 0:
        print(f"[ERROR] RTL compilation failed:\n{out}")
        sys.exit(1)
    print("[PASS] All 15 RTL modules compiled cleanly with zero errors.")

    # Determine test list (support single-test selection from command line)
    target_tests = TESTBENCHES
    if len(sys.argv) > 1:
        requested = sys.argv[1].strip()
        matched = [t for t in TESTBENCHES if t[0] == requested or t[0] == f"tb_{requested}"]
        if matched:
            target_tests = matched
            print(f"[FILTER] Running single requested test: {target_tests[0][0]}")
        else:
            print(f"[WARNING] Requested test '{requested}' not recognized. Running all testbenches.")

    results = []
    total_start = time.time()

    for tb_name, tb_file, req_rtl in target_tests:
        print("-" * 80)
        print(f"[RUNNING] Testbench: {tb_name}")
        tb_log = os.path.join(LOG_DIR, f"{tb_name}.log")

        # Compile testbench
        tb_compile_cmd = [vlog_bin, "-work", "work", tb_file]
        code, out = run_command(tb_compile_cmd, cwd=SIM_DIR)
        if code != 0:
            print(f"[FAIL] {tb_name} compilation failed:\n{out}")
            results.append((tb_name, "FAIL", "Compilation Error", 0.0))
            continue

        # Run simulation in batch mode
        t0 = time.time()
        vsim_cmd = [vsim_bin, "-c", "-do", "run -all; quit -f", f"work.{tb_name}"]
        code, out = run_command(vsim_cmd, cwd=SIM_DIR)
        elapsed = time.time() - t0

        # Save log file
        with open(tb_log, "w", encoding="utf-8") as f:
            f.write(out)

        # Evaluate pass/fail criteria
        passed = ("*** TEST PASSED" in out or "TEST PASSED" in out) and ("[FAIL]" not in out) and ("[FATAL]" not in out)
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
        sys.exit(0)
    else:
        print("[FAILURE] REGRESSION HAD FAILING TESTS")
        sys.exit(1)


if __name__ == "__main__":
    main()
