# Project Results & Verification Evidence: PS-02 PID Flight Controller

This directory serves as the structured archive for all authentic experimental results, verification proof, synthesis schematics, physical implementation layouts, timing reports, and power analyses for the **PS-02 Mode-Switching Hardware PID Flight Controller**.

---

## Directory Organization

```text
results/
├── README.md                      # Master index and engineering summary (this file)
├── synthesis/                     # Logic synthesis schematics, logs & reports
│   ├── README.md                  # Detailed synthesis documentation
│   ├── synthesis_summary.rpt      # ASCII machine-readable synthesis report
│   ├── 01_genus_flow_completion_log.png
│   ├── 02_ps02_top_mapped_schematic.png
│   ├── 03_ps02_top_hierarchical_schematic.png
│   └── 04_schematic_attitude_controller.png ... 09_schematic_ps02_top_complete.png
├── timing/                        # Innovus timing slack & closure reports across stages
│   ├── README.md                  # Timing methodology & margin documentation
│   ├── timing_summary.rpt         # ASCII machine-readable timing report (Pre-CTS to Post-Route)
│   ├── 01_prects_setup_timing.png ... 04_prects_eco_hold_timing.png
│   ├── 05_postcts_setup_timing.png ... 08_postcts_eco_hold_timing.png
│   └── 09_postroute_eco_setup_timing.png ... 11_postroute_eco_command.png
├── power/                         # Power distribution & dissipation reports
│   ├── README.md                  # Power methodology documentation
│   ├── power_summary.rpt          # ASCII machine-readable power dissipation report (3.176 mW)
│   └── 01_power_rail_and_grid.png # Power ring and follow-pin layout
├── physical_design/               # Layout, PnR, DRC & connectivity reports
│   ├── README.md                  # Floorplan, routing & physical verification guide
│   ├── drc_summary.rpt            # ASCII machine-readable DRC report (0 violations)
│   ├── connectivity_summary.rpt   # ASCII machine-readable connectivity report (0 errors, 0 warnings)
│   ├── 01_initial_placement.png
│   ├── 02_clock_tree_synthesis.png
│   ├── 03_routing_in_progress.png
│   ├── 04_layout_after_nanoroute.png
│   ├── 05_final_layout_with_fillers.png
│   ├── 06_verify_drc_zero_violations.png
│   └── 07_verify_connectivity_zero_violations.png
└── simulation/                    # ModelSim / NC-Sim waveforms, logs & co-sim plots
    ├── README.md                  # Verification suite guide and checklist mapping
    ├── simulation_summary.rpt     # ASCII machine-readable functional verification report
    ├── 01_top_level_regression_pass.png ... 11_sensor_to_motor_latency_diagnostic.png
    └── 12_pid_step_response.png ... 19_fsm_transition_testbench_code.png
```

---

## Master PPA & Verification Summary Table

| Category | Primary Metric | Hardware Result | Evidence & Citation |
|:---|:---|:---:|:---|
| **Target Clock** | Frequency / Period | `50.000 MHz` / `20.000 ns` | `constraints/ps02_top.sdc`, [`results/timing/`](timing/) |
| **Setup Timing** | Worst Negative Slack (WNS) | **`+0.016 ns`** (0 Violations) | [`results/timing/09_postroute_eco_setup_timing.png`](timing/09_postroute_eco_setup_timing.png) |
| **Hold Timing** | Worst Negative Slack (WNS) | **`-0.000 ns`** (1 Marginal Path) | [`results/timing/10_postroute_eco_hold_timing.png`](timing/10_postroute_eco_hold_timing.png) |
| **Power Dissipation** | Total Core Power (50 MHz) | **`3.176 mW`** (80.1% Internal) | [`results/power/power_summary.rpt`](power/power_summary.rpt) |
| **Core Boundary** | Dimensions / Area | `446.31 × 444.28 µm` (~0.198 mm²) | [`results/physical_design/05_final_layout_with_fillers.png`](physical_design/05_final_layout_with_fillers.png) |
| **Cell Density** | Final Post-Filler Density | **`74.835%`** (NanoRoute 0.00% overflow)| [`results/physical_design/drc_summary.rpt`](physical_design/drc_summary.rpt) |
| **Design Rule Check** | Geometry DRC Violations | **`0 Violations`** (16 sub-regions) | [`results/physical_design/06_verify_drc_zero_violations.png`](physical_design/06_verify_drc_zero_violations.png) |
| **Net Connectivity** | Shorts / Opens / Floating | **`0 Errors, 0 Warnings`** (~20,000 nets) | [`results/physical_design/07_verify_connectivity_zero_violations.png`](physical_design/07_verify_connectivity_zero_violations.png) |
| **Full-Chip Regression**| Top-Level Assertions | **`13/13 PASSED`** (0 Failures) | [`results/simulation/01_top_level_regression_pass.png`](simulation/01_top_level_regression_pass.png) |
| **Verification Suite** | Testbench Pass Rate | **`8/8 PASSED`** (100% Pass Rate) | [`sim/logs/*.log`](../sim/logs/), [`results/simulation/`](simulation/) |
| **Timing Jitter** | Control Tick Period (1 kHz) | **`0.000 cycles`** (Zero Jitter) | [`results/simulation/08_control_tick_period_jitter_results.png`](simulation/08_control_tick_period_jitter_results.png) |
| **Execution Latency** | Sensor Input to Motor Output | **`1 Clock Cycle`** (Deterministic) | [`results/simulation/11_sensor_to_motor_latency_diagnostic.png`](simulation/11_sensor_to_motor_latency_diagnostic.png) |
| **Closed-Loop Roll** | Overshoot / Settling Time | **`6.73%`** / **`4.57 s`** ($e_{ss}=0.0865°$)| [`results/simulation/16_closed_loop_step_response_metrics.png`](simulation/16_closed_loop_step_response_metrics.png) |

---

## Licensing & Proprietary PDK Boundaries

1. **Machine-Readable Reports vs. Screenshots:** The `.rpt` files in this directory compile and transcribe the quantitative numerical results displayed in the authentic Cadence tool terminal logs and report figures.
2. **Proprietary PDK Files:** In strict adherence to foundry NDA and university licensing terms, raw foundry standard-cell library decks (`.lib`, `.lef`) and binary GDSII files containing proprietary transistor geometries are not redistributed.
3. **Reproducibility:** Complete synthesizable RTL, SDC timing constraints, ModelSim macros, Python test scripts, and Cadence Genus/Innovus automation scripts and templates are provided in the repository to reproduce these exact results under any authorized foundry standard-cell PDK.
