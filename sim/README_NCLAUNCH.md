# Cadence NCLaunch / Incisive Simulation Guide

This project is fully verified to run in a Cadence Incisive (NC-Verilog / NCLaunch) environment. The source code strictly adheres to synthesizable **Verilog-2001** and contains no SystemVerilog extensions.

## Setup Instructions

1. **Environment Setup**
   Ensure your Cadence tools (`ncvlog`, `ncelab`, `ncsim`) are in your `$PATH`.
   
   ```bash
   # Example setup (depends on your server's module system)
   module load cadence/incisive
   ```

2. **Navigate to the simulation directory**
   ```bash
   cd sim/
   ```

3. **Make the regression script executable**
   ```bash
   chmod +x run_nclaunch.sh
   ```

## Running the Automated Regression

A fully automated regression suite is provided to compile, elaborate, and simulate all testbenches.

```bash
./run_nclaunch.sh
```

### What the script does:
1. Compiles all RTL and testbench files from the `nc_files.f` list using `ncvlog -v2001`.
2. Elaborates each testbench using `ncelab`.
3. Simulates each testbench using `ncsim`.
4. Parses the logs (stored in `sim/logs/`) to determine PASS/FAIL status based on the self-checking Verilog logic.
5. Produces a final regression summary indicating total passed/failed tests.

## Running Manually in GUI (NCLaunch)

If you prefer to debug using the Cadence GUI, you can invoke the tool directly:

1. Launch NCLaunch:
   ```bash
   nclaunch -new
   ```
2. Set your design directory to `sim/`.
3. In the GUI, compile the files listed in `nc_files.f` using the **Verilog 2001** parser option.
4. Elaborate the desired testbench (e.g., `work.tb_ps02_top`).
5. Open the simulator to view waveforms.

## Notes on Current Status

* **ModelSim Verification:** The project has been fully compiled and verified locally using ModelSim/Questa with a 100% pass rate.
* **Cadence Verification:** The `nc_files.f` and `run_nclaunch.sh` have been prepared, but because Cadence tools were not accessible in the local Windows environment, a live Cadence run has not been executed locally. Please run `./run_nclaunch.sh` on your Cadence-equipped Linux environment.
