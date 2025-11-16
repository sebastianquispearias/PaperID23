# Leader–Follower Sizing in Heterogeneous UAV Swarms for SAR

This repository contains the code and data used in the paper:

**Leader-Follower Sizing in Heterogeneous UAV Swarms for SAR: Latency–Coverage Trade-Offs via Discrete-Event Simulation**

Authors: Sebastián Quispe Arias, Markus Endler, Bruno José Olivieri de Souza  
Accepted at **LAFUSION 2025** (Paper ID 23).

The code implements a Gradysim-based discrete-event simulation of heterogeneous UAV swarms with Explorer Quad-Copters (EQCs) and Visiting Quad-Copters (VQCs), organized in a leader–follower topology. The experiments explore how the number of leaders **K** and the followers-per-leader ratio **ρ** affect **service latency** and **coverage** under different SAR workloads.

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
  Gradysim behavior for PoIs (targets) used by the simulator.

- `experiments.py`  
  Helper module that defines the **experiment grid** used in the paper  
  (values of K, ρ, P, seeds, and output folders). You can reuse its helpers to script large batches of runs.

- `run_many_seeds.ps1`  
  PowerShell helper script to launch multiple seeds/configurations on Windows  
  (optional, not required to understand or modify the core logic).

Reproducibility data:

- `REPRODUCIBILITY/`  
  - `poi_1000_all_seeds.xlsx` – Raw measurements for workload P = 1000 across all seeds.  
  - `poi_2500_all_seeds.xlsx` – Raw measurements for workload P = 2500.  
  - `poi_4000_all_seeds.xlsx` – Raw measurements for workload P = 4000.  
  - `final_summary.xlsx` – Aggregated results and per-configuration means used for the tables and figures in the paper.

These spreadsheets are exactly the raw data used for Tables 2–4 and Figures 3–5 in the paper.

Folders that may appear after running experiments (example outputs):

- `runs_*`, `poi1000/`, `poi2500/`, `poi4000/`, `logs/` – Logs and per-run outputs.  
  They are **not required** for understanding the code and can be deleted if needed.

---

## Requirements

- **Python** ≥ 3.10
- **Gradysim** (not included in this repo)  
  The simulator used in the paper. This code assumes that the `gradysim` package  
  is already installed and importable (`import gradysim` works).

  If you have access to the Gradysim repository, a typical installation is:

  - `git clone <URL-OF-GRADYSIM>`  
  - `cd gradysim`  
  - `pip install -e .`

Python packages (install via `pip`), for example:

- `pip install numpy pandas scipy matplotlib tqdm openpyxl`

Depending on your environment you may need additional standard packages, but the list above covers the main dependencies used in this repository.

---

## Quick start: run a single simulation

From the repository root, an example run is:

- `python run_simulation.py --seed 123 --num_pois 1000 --num_eqcs 2 --num_vqcs 6 --buffer_size 5 --eqc_speed 6.0 --vqc_speed 12.0 --camera_reach 60.0 --policy load_balancing --no_rt --no_vis`

This corresponds to:

- Workload **P = 1000** PoIs  
- **K = 2** leaders (EQCs)  
- **ρ = 3** followers per leader → `num_vqcs = ρK = 6`  
- Buffer size **M = 5** at each VQC  
- EQC/VQC speeds and camera reach matching the regime described in the paper  
- `--no_rt`, `--no_vis` disable real-time animation and visualization (faster batch runs)

To see all available options:

- `python run_simulation.py --help`

The script prints a summary to stdout and writes a log file with the full configuration and metrics for post-processing.

---

## Reproducing the paper’s experiment grid

The main experiments sweep:

- Workloads: **P ∈ {1000, 2500, 4000}** PoIs  
- Leaders: **K ∈ {1, 2, 3, 4}**  
- Followers-per-leader: **ρ ∈ {1, 2, 3, 4}** → `num_vqcs = ρK`  
- Seeds: **S = 6** per configuration (we used seeds `{123, 124, 125, 126, 127, 128}`)

In code, `experiments.py` defines this grid and how outputs are organized.

If you prefer to run manually (conceptually), for example for `K = 2`, `ρ = 3`, `P = 2500`, seeds 123–128:

For each `seed` in `{123, 124, 125, 126, 127, 128}`, run:

- `python run_simulation.py --seed <seed> --num_pois 2500 --num_eqcs 2 --num_vqcs 6 --buffer_size 5 --eqc_speed 6.0 --vqc_speed 12.0 --camera_reach 60.0 --policy load_balancing --no_rt --no_vis`

On Windows/PowerShell you can adapt the loop or use `run_many_seeds.ps1` as a template.

The resulting logs can then be parsed with your own scripts (or a notebook) to recompute the metrics reported in the paper (mean service latency, ACK delay, end-to-end latency, coverage).

If you only need the **exact numbers** used in the camera-ready version, you can work directly from the Excel files in `REPRODUCIBILITY/`:

- Each `poi_XXXX_all_seeds.xlsx` file contains per-PoI measurements for all seeds at that workload.  
- `final_summary.xlsx` aggregates per-configuration means and is what we used to populate Tables 2–4 and to plot the latency–coverage Pareto fronts.

---

## Adjusting scenarios

You can customize the scenario via:

- `config.py` – Map size, mission horizon, communication radius, default speeds, buffer sizes, etc.  
- CLI arguments in `run_simulation.py` – Override many of the defaults for specific runs (number of PoIs, K, ρ, buffer size, etc.).

Protocol files:

- `eqc_protocol.py` – Leader logic (patrol pattern, assignment policy, timers).  
- `vqc_protocol.py` – Follower logic (navigation, satellite behavior, delivery logic).  
- `poi_protocol.py` – PoI behavior.

These components correspond to the system model and algorithms described in the Method section of the paper.

---

## Reproducibility data

For convenience, the repository includes a minimal reproducibility package in `REPRODUCIBILITY/`:

- Raw, per-PoI measurements for each workload (P = 1000, 2500, 4000).  
- The aggregated summary used to build the tables and figures.

This means that you can fully reconstruct the plots and tables **without rerunning the simulator**, as long as you can read `.xlsx` files (e.g., with Python/pandas, R, or any spreadsheet tool).

---

## Citation

If you use this code or data in academic work, please cite:

`@inproceedings{QuispeArias2025LeaderFollower,
  author    = {Sebasti{\'a}n Quispe Arias and
               Markus Endler and
               Bruno Jos{\'e} Olivieri de Souza},
  title     = {Leader-Follower Sizing in Heterogeneous UAV Swarms for SAR:
               Latency-Coverage Trade-Offs via Discrete-Event Simulation},
  booktitle = {Proceedings of {LAFUSION} 2025},
  year      = {2025},
  note      = {Paper ID 23}
}`
