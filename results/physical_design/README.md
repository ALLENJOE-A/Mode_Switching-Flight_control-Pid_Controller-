# Physical Design & Layout Implementation: PS-02 PID Flight Controller

This directory contains the authentic Cadence Innovus physical design layout screenshots, floorplanning artifacts, CTS buffer network diagrams, NanoRoute traces, and DRC/connectivity verification reports for `ps02_top`.

---

## 1. Summary of Physical Design Artifacts

| Figure / Artifact | Description | Source Reference |
|:---|:---|:---|
| [`01_initial_placement.png`](01_initial_placement.png) | Standard cell placement inside core boundary (`446.31 × 444.28 µm`); initial cell density: 69.886% | Report Fig. 16 |
| [`02_clock_tree_synthesis.png`](02_clock_tree_synthesis.png) | Multi-level balanced clock tree network built by CCOpt; cell density increased to 71.590% | Report Fig. 22 |
| [`03_routing_in_progress.png`](03_routing_in_progress.png) | NanoRoute active global/detailed routing tracks across core area | Report Fig. 27 |
| [`04_layout_after_nanoroute.png`](04_layout_after_nanoroute.png) | Complete routed signal nets with 0.00% horizontal and vertical routing overflow | Report Fig. 28 |
| [`05_final_layout_with_fillers.png`](05_final_layout_with_fillers.png) | Final tapeout-ready layout after core filler cell insertion; final cell density: 74.835% | Report Fig. 32 |
| [`06_verify_drc_zero_violations.png`](06_verify_drc_zero_violations.png) | Cadence Innovus `verifyGeometry` terminal log: 0 violations across 16 sub-regions | Report Fig. 33 |
| [`07_verify_connectivity_zero_violations.png`](07_verify_connectivity_zero_violations.png) | Cadence Innovus `verifyConnectivity` terminal log: 0 problems or warnings across ~20,000 nets | Report Fig. 34 |
| [`drc_summary.rpt`](drc_summary.rpt) | ASCII machine-readable DRC report extracting geometry verification results | Tool Extraction |
| [`connectivity_summary.rpt`](connectivity_summary.rpt) | ASCII machine-readable connectivity report extracting electrical net verification | Tool Extraction |

---

## 2. Floorplan & Implementation Metrics

- **Die Boundary:** `(0.000, 0.000) to (446.310, 444.280) µm`
- **Total Silicon Area:** `~0.1983 mm²` (`198,287.6 µm²`)
- **Initial Placement Density:** `69.886%`
- **Post-CTS Density:** `71.590%`
- **Final Routed Density (with Fillers):** `74.835%`
- **Routing Overflow:** `0.00% Horizontal, 0.00% Vertical` (~21,600 nets)
- **Internal DRC Cleanliness:** `0 Violations` (across 16 sub-regions)
- **Internal Connectivity Cleanliness:** `0 Errors, 0 Warnings` (across ~20,000 nets)

> [!NOTE]
> Proprietary foundry GDSII streams and restricted LEF technology decks cannot be redistributed in public repositories. Reproducible place-and-route configuration templates are provided in `scripts/physical_design/innovus_pnr.tcl` and `innovus_config.tcl.template`.
