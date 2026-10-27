# Simulation Results, Waveforms & Verification: PS-02 PID Flight Controller

This directory contains the authentic ModelSim and Cadence NC-Sim simulation waveform captures, regression summary outputs, golden model comparison logs, and closed-loop flight dynamics step-response plots.

---

## 1. Summary of Simulation Artifacts

| Figure / Artifact | Description | Source Reference |
|:---|:---|:---|
| [`01_top_level_regression_pass.png`](01_top_level_regression_pass.png) | Top-level integration regression summary: 13 passed checks, 0 failed | Report Fig. 2 |
| [`02_regression_checklist_summary.png`](02_regression_checklist_summary.png) | Regression checklist status showing all verified operational modes and safety policies | Report Fig. 3 |
| [`03_per_module_log_status.png`](03_per_module_log_status.png) | Log grep across all 8 testbenches verifying 0 errors and clean pass status | Report Fig. 4 |
| [`04_pid_golden_model_mismatches.png`](04_pid_golden_model_mismatches.png) | ModelSim verification log during testbench development showing golden model comparison | Report Fig. 5 |
| [`05_pid_golden_model_pass.png`](05_pid_golden_model_pass.png) | Final golden model test pass log: all vectors matched | Report Fig. 6 |
| [`06_pid_controller_waveform.png`](06_pid_controller_waveform.png) | ModelSim waveform display showing P, I, D terms and actuator command response | Report Fig. 7 |
| [`07_mode_switching_failsafe_waveform.png`](07_mode_switching_failsafe_waveform.png) | Waveform demonstrating glitch-free ACRO to STABILIZE transition and watchdog failsafe trip | Report Fig. 8 |
| [`08_control_tick_period_jitter_results.png`](08_control_tick_period_jitter_results.png) | Quantitative period measurement verifying 50,000 cycles per tick and zero jitter | Report Fig. 9 |
| [`09_tick_rate_jitter_waveform.png`](09_tick_rate_jitter_waveform.png) | SimVision / ModelSim waveform of control tick strobe periodicity | Report Fig. 10 |
| [`10_sensor_to_motor_timing_waveform.png`](10_sensor_to_motor_timing_waveform.png) | Cycle-accurate sensor input to motor output latency waveform | Report Fig. 11 |
| [`11_sensor_to_motor_latency_diagnostic.png`](11_sensor_to_motor_latency_diagnostic.png) | Diagnostic output measuring pipeline execution latency (1 cycle) | Report Fig. 12 |
| [`12_pid_step_response.png`](12_pid_step_response.png) | Dynamic step response plot showing individual P, I, D term contributions | Report Fig. 35 |
| [`13_pid_output_response.png`](13_pid_output_response.png) | Closed-loop PID controller output trajectory | Report Fig. 36 |
| [`14_roll_angle_step_response.png`](14_roll_angle_step_response.png) | Closed-loop roll-angle step response against quadcopter plant model | Report Fig. 37 |
| [`15_tracking_error_response.png`](15_tracking_error_response.png) | Attitude tracking error convergence over time | Report Fig. 38 |
| [`16_closed_loop_step_response_metrics.png`](16_closed_loop_step_response_metrics.png) | Metrics file: Overshoot 6.73%, Settling Time 4.57 s, Steady-State Error 0.0865° | Report Fig. 39 |
| [`17_closed_loop_roll_response.png`](17_closed_loop_roll_response.png) | Extended closed-loop roll trajectory under disturbance | Report Fig. 40 |
| [`18_pid_reference_csv_preview.png`](18_pid_reference_csv_preview.png) | Preview of numerical test vector reference dataset | Report Reference |
| [`19_fsm_transition_testbench_code.png`](19_fsm_transition_testbench_code.png) | Verilog testbench implementation of FSM transition verification | Report Reference |
| [`simulation_summary.rpt`](simulation_summary.rpt) | ASCII machine-readable report summarizing regression status, latency, and flight metrics | Tool Extraction |

---

## 2. Reproduction Instructions

To run the full verification regression locally:
```bash
python scripts/run_regression.py
```
To run any specific testbench individually:
```bash
python scripts/run_regression.py tb_pid_controller
```
All simulation execution logs are maintained in `sim/logs/`.
