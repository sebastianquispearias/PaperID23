# Leader–Follower Sizing in Heterogeneous UAV Swarms for SAR

This repository contains the code and data used in the paper:

**Leader-Follower Sizing in Heterogeneous UAV Swarms for SAR: Latency–Coverage Trade-Offs via Discrete-Event Simulation**

Sebastián Quispe Arias, Markus Endler, Bruno José Olivieri de Souza  
Accepted at **LAFUSION 2025** (Paper ID 23).

The code implements a discrete-event simulation (built on top of *Gradysim*) of heterogeneous UAV swarms with Explorer Quad-Copters (EQCs) and Visiting Quad-Copters (VQCs), organized in a leader–follower topology. The experiments explore how the number of leaders K and the followers-per-leader ratio rho affect **service latency** and **coverage** under different SAR workloads.

---

## Repository layout

**Main files**

- `run_simulation.py`  
  Entry point to run a single simulation for a given configuration  
  (`num_pois`, `num_eqcs = K`, `num_vqcs = rho * K`, buffer size, speeds, etc.).

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
  (values of K, rho, P, seeds, and output folders).  
  You can reuse its helpers to script large batches of runs if needed.

- `run_many_seeds.ps1`  
  PowerShell helper script to launch multiple seeds / configurations on Windows  
  (optional, not required to understand or modify the core logic).

**Reproducibility data**

- `REPRODUCIBILITY/`
  - `poi_1000_all_seeds.xlsx` – Raw measurements for workload P = 1000 across all seeds.  
  - `poi_2500_all_seeds.xlsx` – Raw measurements for workload P = 2500.  
  - `poi_4000_all_seeds.xlsx` – Raw measurements for workload P = 4000.  
  - `final_summary.xlsx` – Aggregated results and per-configuration means used for the tables and figures in the paper.

These spreadsheets are exactly the **raw data** used for Tables 2–4 and Figures 3–5.

**Folders that may appear after running experiments**

- `runs_*`, `poi1000/`, `poi2500/`, `poi4000/`, `logs/` – Example outputs and logs.  
  They are not required for understanding the code and can be deleted if needed.

---

## Requirements

- **Python** ≥ 3.10
- **Gradysim** (not included in this repository)  
  This code assumes that the `gradysim` package is already installed and importable (i.e., `import gradysim` works).  
  Please install Gradysim from your local source or internal repository according to its own instructions.

**Python packages** (install via `pip`):

```bash
pip install numpy pandas scipy matplotlib tqdm openpyxl
Depending on your environment you may need additional standard packages, but the above cover the main dependencies of the scripts in this repo.

Quick start: run a single simulation
From the repository root, run for example:

bash
Copiar código
python run_simulation.py \
  --seed 123 \
  --num_pois 1000 \
  --num_eqcs 2 \
  --num_vqcs 6 \
  --buffer_size 5 \
  --eqc_speed 6.0 \
  --vqc_speed 12.0 \
  --camera_reach 60.0 \
  --policy load_balancing \
  --no_rt \
  --no_vis
This corresponds to:

Workload P = 1000 PoIs

K = 2 leaders (EQCs)

rho = 3 followers per leader → num_vqcs = rho * K = 6

Buffer size M = 5 at each VQC

EQC/VQC speeds and camera reach matching the regime described in the paper

--no_rt, --no_vis disable real-time animation and visualization (faster batch runs)

To see all available options, run:

bash
Copiar código
python run_simulation.py --help
The script prints a summary to stdout and writes a log file with the full configuration and metrics for post-processing.

Reproducing the paper’s experiment grid
The main experiments sweep:

Workloads: P ∈ {1000, 2500, 4000} PoIs

Leaders: K ∈ {1, 2, 3, 4}

Followers-per-leader: rho ∈ {1, 2, 3, 4} → num_vqcs = rho * K

Seeds: S = 6 seeds per configuration (paper uses seeds {123, 124, 125, 126, 127, 128})

In code, experiments.py defines this grid and how outputs are organized.

If you prefer to run by hand, you can do something like:

bash
Copiar código
# Example: K = 2, rho = 3, P = 2500, seeds 123–128
for seed in 123 124 125 126 127 128; do
  python run_simulation.py \
    --seed "$seed" \
    --num_pois 2500 \
    --num_eqcs 2 \
    --num_vqcs 6 \
    --buffer_size 5 \
    --eqc_speed 6.0 \
    --vqc_speed 12.0 \
    --camera_reach 60.0 \
    --policy load_balancing \
    --no_rt \
    --no_vis
done
On Windows/PowerShell you can adapt the loop or use the provided run_many_seeds.ps1 script as a template.

The resulting logs can be parsed with your own scripts (or a notebook) to recompute the metrics reported in the paper:

mean service latency

ACK delay

end-to-end latency

coverage

If you just want the exact numbers used in the camera-ready, you can work directly from the Excel files in REPRODUCIBILITY/:

Each poi_XXXX_all_seeds.xlsx file contains per-PoI measurements for all seeds at that workload.

final_summary.xlsx aggregates per-configuration means and is what we used to populate Tables 2–4 and to plot the latency–coverage Pareto fronts.

Adjusting scenarios
You can customize the scenario via:

config.py
Map size, mission horizon, communication radius, default speeds, buffer sizes, etc.

CLI arguments in run_simulation.py
Override many of the defaults for specific runs (number of PoIs, K, rho, buffer size, etc.).

Protocol files:

eqc_protocol.py – leader logic (patrol pattern, assignment policy, timers)

vqc_protocol.py – follower logic (navigation, satellite behavior, delivery logic)

poi_protocol.py – PoI behavior

These are the same components described in the Method section of the paper.

Reproducibility data
For convenience, the repository includes a minimal reproducibility package in REPRODUCIBILITY/:

Raw, per-PoI measurements for each workload (P = 1000, 2500, 4000)

The aggregated summary used to build the tables and figures

This means that you can fully reconstruct the plots and tables even if you do not re-run the simulator, as long as you can read .xlsx files (e.g., with Python/pandas, R, or any spreadsheet tool).

Citation
If you use this code or data in academic work, please cite the paper:

bibtex
Copiar código
@inproceedings{QuispeArias2025LeaderFollower,
  author    = {Sebasti{\'a}n Quispe Arias and
               Markus Endler and
               Bruno Jos{\'e} Olivieri de Souza},
  title     = {Leader-Follower Sizing in Heterogeneous UAV Swarms for SAR:
               Latency-Coverage Trade-Offs via Discrete-Event Simulation},
  booktitle = {Proceedings of {LAFUSION} 2025},
  year      = {2025},
  note      = {Paper ID 23}
}