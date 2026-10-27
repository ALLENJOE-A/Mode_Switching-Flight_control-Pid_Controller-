# Synthesis Results & Schematics: PS-02 PID Flight Controller

This directory contains the authentic Cadence Genus synthesis logs, gate-level mapped schematics, sub-block structural diagrams, and synthesis summary reports.

---

## 1. Summary of Synthesis Evidence

| Figure / Artifact | Description | Source Reference |
|:---|:---|:---|
| [`01_genus_flow_completion_log.png`](01_genus_flow_completion_log.png) | Cadence Genus terminal log demonstrating synthesis flow completion, netlist export, and SDC generation | Report Fig. 13 |
| [`02_ps02_top_mapped_schematic.png`](02_ps02_top_mapped_schematic.png) | Technology-mapped gate-level schematic view of `ps02_top` showing mapped standard cell logic | Report Fig. 14 |
| [`03_ps02_top_hierarchical_schematic.png`](03_ps02_top_hierarchical_schematic.png) | Hierarchical schematic viewer showing top-level module interconnects | Report Fig. 15 |
| [`04_schematic_attitude_controller.png`](04_schematic_attitude_controller.png) | Detailed mapped schematic of `attitude_controller` (Roll/Pitch outer loops) | EDA Schematics |
| [`05_schematic_rate_controller.png`](05_schematic_rate_controller.png) | Detailed mapped schematic of `rate_controller` (3-axis inner rate loops) | EDA Schematics |
| [`06_schematic_mode_controller.png`](06_schematic_mode_controller.png) | Detailed mapped schematic of `mode_controller` (FSM transitions & overrides) | EDA Schematics |
| [`07_schematic_gain_bank.png`](07_schematic_gain_bank.png) | Detailed mapped schematic of `gain_bank` (per-mode atomic PID gain storage) | EDA Schematics |
| [`08_schematic_failsafe_controller.png`](08_schematic_failsafe_controller.png) | Detailed mapped schematic of `failsafe_controller` (25% descent engine) | EDA Schematics |
| [`09_schematic_ps02_top_complete.png`](09_schematic_ps02_top_complete.png) | Complete gate-level full-chip schematic view of `ps02_top` | EDA Schematics |
| [`synthesis_summary.rpt`](synthesis_summary.rpt) | ASCII machine-readable report extracting synthesis parameters, constraints, and hierarchy | Tool Extraction |

---

## 2. Key Synthesis Parameters

- **Target Clock:** 50.00 MHz (`T = 20.0 ns`), input jitter/uncertainty = `0.20 ns`.
- **Clock Tree Latency:** Estimated insertion delay = `0.50 ns`.
- **Operating Library:** Generic digital standard cell library (Cadence 90nm flow mode).
- **Hierarchy:** 17 modules fully synthesized and flattened at the standard cell level without inferred latches or combinational loops.
- **Interconnect Scale:** ~21,600 nets, 1,118 timing paths.
- **Power Optimization:** Vectorless statistical activity estimation.

> [!NOTE]
> Foundry technology characterization libraries (`.lib`) and proprietary mapped gate-level netlists containing foundry primitives are restricted from public distribution under non-disclosure and licensing agreements. The complete synthesizable RTL, SDC constraints, and reproducible Genus TCL scripts are provided in `rtl/`, `constraints/`, and `scripts/synthesis/`.
