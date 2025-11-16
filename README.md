# Leader–Follower Sizing in Heterogeneous UAV Swarms for SAR

This repository contains the code and data used in the paper:

**Leader-Follower Sizing in Heterogeneous UAV Swarms for SAR: Latency–Coverage Trade-Offs via Discrete-Event Simulation**

Sebastián Quispe Arias, Markus Endler, Bruno José Olivieri de Souza  

Accepted at **LAFUSION 2025** (Paper ID 23).

The code implements a Gradysim-based discrete-event simulation of heterogeneous UAV swarms with Explorer Quad-Copters (EQCs) and Visiting Quad-Copters (VQCs), organized in a leader–follower topology. The experiments explore how the number of leaders \(K\) and the followers-per-leader ratio \(\rho\) affect **service latency** and **coverage** under different SAR workloads.

---

## Repository layout

Main files:

- `run_simulation.py`  
  Entry point to run a single simulation for a given configuration  
  (`num_pois`, `num_eqcs = K`, `num_vqcs = ρK`, buffer size, speeds, etc.).

- `config.py`  
  Central configuration for map size, speeds, time horizon, communication radius, and experiment defaults.

- `eqc_protocol.py`  
  Gradysim behavior for **Explorer Quad-Copters (EQCs)**: patrol, PoI detection, task assignment, and ACK handling.

- `vqc_protocol.py`  
  Gradysim behavior for **Visiting Quad-Copters (VQCs)**: task execution, satellite loitering around the leader, and delivery of PoI confirmations.

- `poi_protocol.py`  
  Simple Gradysim behavior for PoIs (targets) used by the simulator.

- `experiments.py`  
  Helper module that defines the **experiment grid** used in the paper  
  (values of \(K\), \(\rho\), \(P\), seeds, and output folders).  
  You can reuse its helpers to script large batches of runs if needed.

- `run_many_seeds.ps1`  
  PowerShell helper script to launch multiple seeds/configurations on Windows  
  (optional, not required to understand or modify the core logic).

Reproducibility data:

- `REPRODUCIBILITY/`
  - `poi_1000_all_seeds.xlsx` – Raw measurements for workload \(P = 1000\) across all seeds.  
  - `poi_2500_all_seeds.xlsx` – Raw measurements for workload \(P = 2500\).  
  - `poi_4000_all_seeds.xlsx` – Raw measurements for workload \(P = 4000\).  
  - `final_summary.xlsx` – Aggregated results and per-configuration means used for the tables and figures in the paper.

These spreadsheets are exactly the **raw data** used for Tables 2–4 and Figures 3–5.

Folders that may appear after running experiments:

- `runs_*`, `poi1000/`, `poi2500/`, `poi4000/`, `logs/` – Example outputs and logs.  
  They are not required for understanding the code and can be deleted if needed.

---

## Requirements

- **Python** ≥ 3.10
- **Gradysim** (external simulator, not included in this repository)  
  This code assumes that the `gradysim` package is already installed and importable (e.g. `import gradysim` works).  
  If you have access to the Gradysim repository, follow its README to install it (typically `pip install -e .` from the Gradysim repo).

Python packages (install via `pip`):

```bash
pip install numpy pandas scipy matplotlib tqdm openpyxl








