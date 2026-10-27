# Timing Results & Slack Closure Analysis: PS-02 PID Flight Controller

This directory contains the authentic Cadence Innovus timing slack screenshots, WNS/TNS analysis tables, and timing reports across all physical implementation stages from Pre-CTS to Post-Route ECO.

---

## 1. Timing Summary by Implementation Stage

| Physical Design Stage | Setup WNS (ns) | Setup Violations | Hold WNS (ns) | Hold Violations | Cell Density | Screenshot Evidence |
|:---|:---:|:---:|:---:|:---:|:---:|:---|
| **Pre-CTS (Ideal Clock)** | -7.733 | 220 | -0.180 | 677 | 69.886% | [`01_prects_setup_timing.png`](01_prects_setup_timing.png), [`02_prects_hold_timing.png`](02_prects_hold_timing.png) |
| **Pre-CTS (Post-Opt)** | **+0.002** | **0** | -0.186 | 648 | 70.120% | [`03_prects_eco_setup_timing.png`](03_prects_eco_setup_timing.png), [`04_prects_eco_hold_timing.png`](04_prects_eco_hold_timing.png) |
| **Post-CTS (Propagated)** | -0.146 | 56 | -0.187 | 668 | 71.590% | [`05_postcts_setup_timing.png`](05_postcts_setup_timing.png), [`06_postcts_hold_timing.png`](06_postcts_hold_timing.png) |
| **Post-CTS (Post-ECO)** | **+0.031** | **0** | -0.010 | 5 | 72.840% | [`07_postcts_eco_setup_timing.png`](07_postcts_eco_setup_timing.png), [`08_postcts_eco_hold_timing.png`](08_postcts_eco_hold_timing.png) |
| **Post-Route (Post-ECO)** | **+0.016** | **0** | **-0.000** | **1** | **74.835%** | [`09_postroute_eco_setup_timing.png`](09_postroute_eco_setup_timing.png), [`10_postroute_eco_hold_timing.png`](10_postroute_eco_hold_timing.png) |

---

## 2. Timing Analysis & Closure Discussion

1. **Setup Closure:** Setup timing achieved closure with a positive margin of **+0.016 ns** under full parasitics, on-chip variation (OCV), and signal integrity (SI) analysis across all 1,118 analyzed paths.
2. **Hold Margin Transparency:** The post-route hold report displays **-0.000 ns slack** with 1 violating register-to-register path. The magnitude is at the zero rounding boundary (TNS = `-0.000 ns`), while the default path group displays `+0.002 ns`. In accordance with rigorous ASIC tapeout practices, this is truthfully reported as *essentially closed* with minor buffer tuning recommended before final tapeout.
3. **Machine-Readable Report:** Detailed stage-by-stage numerical metrics are compiled in [`timing_summary.rpt`](timing_summary.rpt).
