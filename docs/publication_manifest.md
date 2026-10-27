# Publication Manifest: PS-02 Mode-Switching Hardware PID Controller

**Repository:** `ALLENJOE-A/Mode_Switching-Flight_control-Pid_Controller-`  
**Target Branch:** `main`  
**Authors:** Allen Joe A (24BVD1100), T Venkata Sai Neeraj (24BVD1124)  
**Date:** October 2026  

---

## Candidate Classification

All candidate files have been classified into four rigorous engineering categories:

- **Category A (Required):** Core synthesizable RTL, self-checking testbenches, timing constraints, reproducible automation scripts, root documentation, Git configuration, and license.
- **Category B (Valuable Supporting Evidence):** Verified simulation logs, Cadence Genus mapped schematics, Cadence Innovus physical layout screenshots, PPA timing reports, and closed-loop performance charts.
- **Category C (Exclude / Regenerate):** Transient compiled simulation databases (`sim/work/`), temporary logs (`sim/transcript`), and accidental nested clone directory (`PS02_Mode_Switching_PID_Controller/`).
- **Category D (Never Publish / Out of Scope):** Unrelated workspace folders (`5T OTA`, assignment reports) and proprietary foundry PDK files.

---

## Detailed File Manifest

| Relative Path | Category | Classification Reason | Size (Bytes) | Git Storage Mode |
|:---|:---:|:---|:---:|:---:|
| `README.md` | A | Root technical repository documentation and usage guide | ~20 KB | Direct Git Blob |
| `LICENSE` | A | Open-source MIT License terms | ~1.1 KB | Direct Git Blob |
| `.gitignore` | A | Excludes transient simulation and compiler artifacts | ~600 B | Direct Git Blob |
| `rtl/ps02_top.v` | A | Top-level system integration module | 19,801 | Direct Git Blob |
| `rtl/attitude_controller.v` | A | Cascaded attitude outer-loop controller (Roll/Pitch P) | 4,275 | Direct Git Blob |
| `rtl/rate_controller.v` | A | 3-axis angular rate inner-loop controller (Roll/Pitch/Yaw) | 6,477 | Direct Git Blob |
| `rtl/pid_controller.v` | A | Unified PID controller engine with anti-windup & filter | 7,770 | Direct Git Blob |
| `rtl/mode_controller.v` | A | FSM mode-switching manager (Acro, Stabilize, AltHold, Failsafe) | 4,153 | Direct Git Blob |
| `rtl/bumpless_transfer.v` | A | Integrator state recalculation across mode boundaries | 3,654 | Direct Git Blob |
| `rtl/anti_windup.v` | A | Conditional integration and clamping anti-windup logic | 3,022 | Direct Git Blob |
| `rtl/derivative_filter.v` | A | First-order low-pass IIR derivative filter on measurement | 3,557 | Direct Git Blob |
| `rtl/gain_bank.v` | A | Per-mode PID gain storage with atomic load logic | 13,869 | Direct Git Blob |
| `rtl/config_registers.v` | A | Synchronous memory-mapped configuration register file | 4,156 | Direct Git Blob |
| `rtl/control_tick_generator.v` | A | Selectable 1/2/4/8 kHz control-tick strobe generator | 2,763 | Direct Git Blob |
| `rtl/watchdog_timer.v` | A | Programmable hardware watchdog for sensor dropout detection | 1,958 | Direct Git Blob |
| `rtl/failsafe_controller.v` | A | Emergency descent policy engine on sensor timeout | 3,518 | Direct Git Blob |
| `rtl/actuator_interface.v` | A | Quadcopter X-configuration motor mixer matrix | 3,644 | Direct Git Blob |
| `rtl/fixed_point_saturation.v` | A | Signed Q8.8 saturating arithmetic units (`sat_mult`, `sat_add`) | 3,258 | Direct Git Blob |
| `tb/tb_ps02_top.v` | A | Top-level full-chip self-checking integration testbench | 21,405 | Direct Git Blob |
| `tb/tb_pid_controller.v` | A | Unit testbench for PID arithmetic and dynamic response | 6,670 | Direct Git Blob |
| `tb/tb_mode_controller.v` | A | Unit testbench for mode transitions and priority | 6,111 | Direct Git Blob |
| `tb/tb_bumpless_transfer.v` | A | Unit testbench verifying smooth actuator transfer | 5,228 | Direct Git Blob |
| `tb/tb_anti_windup.v` | A | Unit testbench for saturation clamping and back-calculation | 7,088 | Direct Git Blob |
| `tb/tb_derivative_filter.v` | A | Unit testbench for derivative low-pass attenuation | 5,521 | Direct Git Blob |
| `tb/tb_saturation.v` | A | Unit testbench for extreme arithmetic boundary conditions | 6,769 | Direct Git Blob |
| `tb/tb_watchdog_failsafe.v` | A | Unit testbench for sensor timeout and failsafe activation | 8,091 | Direct Git Blob |
| `sim/nc_files.f` | A | Cadence Incisive file list argument file | 592 | Direct Git Blob |
| `sim/README_NCLAUNCH.md` | A | Cadence Incisive / NCLaunch setup and execution guide | 2,099 | Direct Git Blob |
| `sim/run_all.do` | A | ModelSim compilation and batch execution macro | 1,947 | Direct Git Blob |
| `sim/run_nclaunch.sh` | A | Shell wrapper for Cadence NCLaunch batch execution | 2,660 | Direct Git Blob |
| `sim/logs/*.log` (8 files) | B | Verifiable simulation logs (all 8 tests PASS, 0 errors) | ~13 KB | Direct Git Blobs |
| `scripts/run_modelsim.bat` | A | Windows one-click batch script for ModelSim regression | 751 | Direct Git Blob |
| `scripts/run_regression.py` | A | Portable Python automated test runner with simulator autodetect | ~7.5 KB | Direct Git Blob |
| `constraints/ps02_top.sdc` | A | SDC timing constraints targeting 50 MHz operation | ~2.5 KB | Direct Git Blob |
| `scripts/synthesis/genus_synth.tcl` | A | Reproducible Cadence Genus synthesis script | ~4.0 KB | Direct Git Blob |
| `scripts/synthesis/genus_config.tcl.template` | A | Template for user PDK library path configuration | ~1.5 KB | Direct Git Blob |
| `scripts/physical_design/innovus_pnr.tcl` | A | Reproducible Cadence Innovus place-and-route flow script | ~7.0 KB | Direct Git Blob |
| `scripts/physical_design/innovus_config.tcl.template` | A | Template for user LEF, tech, & rule configuration | ~2.0 KB | Direct Git Blob |
| `docs/architecture.md` | A | Detailed architectural block specifications | 10,544 | Direct Git Blob |
| `docs/fixed_point_design.md` | A | Signed Q8.8 fixed-point arithmetic specification | 5,769 | Direct Git Blob |
| `docs/interface_specification.md` | A | Port list and memory-mapped register map | 8,764 | Direct Git Blob |
| `docs/reference_repository_analysis.md` | A | Analysis and architectural diff vs HiL reference | 8,544 | Direct Git Blob |
| `docs/requirements_traceability.md` | A | Requirements traceability matrix against PS-02 spec | 3,583 | Direct Git Blob |
| `docs/verification_plan.md` | A | Verification methodology and testbench matrix | 5,056 | Direct Git Blob |
| `docs/physical_design.md` | B | Comprehensive physical design, timing, and PPA report | ~10 KB | Direct Git Blob |
| `docs/publication_manifest.md` | A | This document (audit & classification manifest) | ~4 KB | Direct Git Blob |
| `docs/figures/**` | B | Verified layout, schematic, and waveform screenshot evidence | ~7.8 MB | Direct Git Blobs |
| `sim/work/*` | C | Transient ModelSim compiled library database | ~1.0 MB | Excluded (`.gitignore`) |
| `sim/transcript` | C | Transient run log overwritten on every simulation | 3,288 | Excluded (`.gitignore`) |
| `PS02_Mode_Switching_PID_Controller/*`| C | Accidental nested clone directory | ~1.2 MB | Excluded (`.gitignore`) |
| `5T OTA` | D | Unrelated project directory | — | Excluded (Out of Scope) |
| `Silicon_2400_outputs` | D | External reference workspace | — | Excluded (Untouched) |
| `scripts/*/*_config.tcl` | D | Local user PDK path definitions | — | Excluded (`.gitignore`) |
| `*.lib`, `*.lef`, `*.gds` | D | Proprietary foundry technology collateral | — | Excluded (`.gitignore`) |
