# Power Dissipation & Power Grid Distribution: PS-02 PID Flight Controller

This directory contains the power grid structure screenshot, power estimation summary, and analysis report for the `ps02_top` ASIC.

---

## 1. Summary of Power Artifacts

| Figure / Artifact | Description | Source Reference |
|:---|:---|:---|
| [`01_power_rail_and_grid.png`](01_power_rail_and_grid.png) | Cadence Innovus layout screenshot displaying initial core power rings and horizontal Metal1 follow-pin rails | Report Fig. 17 |
| [`power_summary.rpt`](power_summary.rpt) | ASCII machine-readable power dissipation report detailing dynamic, internal, and leakage power | Tool Extraction |

---

## 2. Power Architecture & Metrics Summary

- **Supply Voltage:** 1.0 V nominal.
- **Operating Frequency:** 50.0 MHz.
- **Power Structure:** Dual core power ring (`Metal4` / `Metal5`) with standard cell row follow-pins on `Metal1`.
- **Total Power Estimation:** **3.176 mW**
  - **Internal Power:** `2.544 mW` (80.1%)
  - **Switching Power:** `0.632 mW` (19.9%)
  - **Leakage Power:** `0.12 µW` (< 0.01%)
- **Analysis Methodology:** Vectorless statistical toggle estimation in Cadence Genus / Innovus.
