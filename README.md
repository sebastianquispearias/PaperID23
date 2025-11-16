\# Leader–Follower Sizing in Heterogeneous UAV Swarms for SAR



This repository contains the code and data used in the paper:



> \*\*Leader-Follower Sizing in Heterogeneous UAV Swarms for SAR: Latency–Coverage Trade-Offs via Discrete-Event Simulation\*\*  

> Sebastián Quispe Arias, Markus Endler, Bruno José Olivieri de Souza  

> Accepted at \*\*LAFUSION 2025\*\* (Paper ID 23).



The code implements a \*\*Gradysim-based\*\* discrete-event simulation of heterogeneous UAV swarms with Explorer Quad-Copters (EQCs) and Visiting Quad-Copters (VQCs), organized in a leader–follower topology. The experiments explore how the number of leaders `K` and the followers-per-leader ratio `ρ` affect \*\*service latency\*\* and \*\*coverage\*\* under different SAR workloads.



---



\## Repository layout



Main files:



\- `run\_simulation.py`  

&nbsp; Entry point to run a single simulation for a given configuration  

&nbsp; (`num\_pois`, `num\_eqcs = K`, `num\_vqcs = ρK`, buffer size, speeds, etc.).



\- `config.py`  

&nbsp; Central configuration for map size, speeds, time horizon, communication radius, and experiment defaults.



\- `eqc\_protocol.py`  

&nbsp; Gradysim behavior for \*\*Explorer Quad-Copters (EQCs)\*\*: patrol, PoI detection, task assignment, and ACK handling.



\- `vqc\_protocol.py`  

&nbsp; Gradysim behavior for \*\*Visiting Quad-Copters (VQCs)\*\*: task execution, satellite loitering around the leader, and delivery of PoI confirmations.



\- `poi\_protocol.py`  

&nbsp; Simple Gradysim behavior for PoIs (targets) used by the simulator.



\- `experiments.py`  

&nbsp; Helper module that defines the \*\*experiment grid\*\* used in the paper  

&nbsp; (values of `K`, `ρ`, `P`, seeds, and output folders).  

&nbsp; You can reuse its helpers to script large batches of runs if needed.



\- `run\_many\_seeds.ps1`  

&nbsp; PowerShell helper script to launch multiple seeds / configurations on Windows

&nbsp; (optional, not required to understand or modify the core logic).



Reproducibility data:



\- `REPRODUCIBILITY/`

&nbsp; - `poi\_1000\_all\_seeds.xlsx` – Raw measurements for workload `P = 1000` across all seeds.  

&nbsp; - `poi\_2500\_all\_seeds.xlsx` – Raw measurements for workload `P = 2500`.  

&nbsp; - `poi\_4000\_all\_seeds.xlsx` – Raw measurements for workload `P = 4000`.  

&nbsp; - `final\_summary.xlsx` – Aggregated results and per-configuration means used for the tables and figures in the paper.  



These spreadsheets are exactly the \*\*raw data\*\* used for Tables 2–4 and Figures 3–5.



Folders that may appear after running experiments:



\- `runs\_\*`, `poi1000/`, `poi2500/`, `poi4000/`, `logs/` – Example outputs and logs.  

&nbsp; They are not required for understanding the code and can be deleted if needed.



---



\## Requirements



\- \*\*Python\*\* ≥ 3.10

\- \[Gradysim](https://github.com/)\*\*(not included here)\*\*  

&nbsp; The simulator used in the paper. This code assumes that the `gradysim` package  

&nbsp; is already installed and importable (e.g. `import gradysim` works).



&nbsp; If you have access to the Gradysim repository, a typical installation is:



&nbsp; ```bash

&nbsp; git clone <gradysim-repo-url>

&nbsp; cd gradysim

&nbsp; pip install -e .

Python packages (install via pip):



bash

Copiar código

pip install numpy pandas scipy matplotlib tqdm openpyxl

Depending on your environment you may need additional standard packages, but the

above cover the main dependencies of the scripts in this repo.



Quick start: run a single simulation

From the repository root:



bash

Copiar código

python run\_simulation.py \\

&nbsp; --seed 123 \\

&nbsp; --num\_pois 1000 \\

&nbsp; --num\_eqcs 2 \\

&nbsp; --num\_vqcs 6 \\

&nbsp; --buffer\_size 5 \\

&nbsp; --eqc\_speed 6.0 \\

&nbsp; --vqc\_speed 12.0 \\

&nbsp; --camera\_reach 60.0 \\

&nbsp; --policy load\_balancing \\

&nbsp; --no\_rt \\

&nbsp; --no\_vis

This corresponds to:



Workload P = 1000 PoIs



K = 2 leaders (EQCs)



ρ = 3 followers per leader → num\_vqcs = ρK = 6



Buffer size M = 5 at each VQC



EQC/VQC speeds and camera reach matching the regime described in the paper



--no\_rt, --no\_vis disable real-time animation and visualization (faster batch runs)



To see all available options, run:



bash

Copiar código

python run\_simulation.py --help

The script prints a summary to stdout and writes a log file with the full

configuration and metrics for post-processing.



Reproducing the paper’s experiment grid

The main experiments sweep:



Workloads: P ∈ {1000, 2500, 4000} PoIs



Leaders: K ∈ {1, 2, 3, 4}



Followers-per-leader: ρ ∈ {1, 2, 3, 4} → num\_vqcs = ρK



Seeds: S = 6 seeds per configuration (paper uses seeds {123, 124, 125, 126, 127, 128})



In code, experiments.py defines this grid and how outputs are organized.

If you prefer to run by hand, you can do something like:



bash

Copiar código

\# Example: K=2, ρ=3, P=2500, seeds 123–128

for seed in 123 124 125 126 127 128; do

&nbsp; python run\_simulation.py \\

&nbsp;   --seed "$seed" \\

&nbsp;   --num\_pois 2500 \\

&nbsp;   --num\_eqcs 2 \\

&nbsp;   --num\_vqcs 6 \\

&nbsp;   --buffer\_size 5 \\

&nbsp;   --eqc\_speed 6.0 \\

&nbsp;   --vqc\_speed 12.0 \\

&nbsp;   --camera\_reach 60.0 \\

&nbsp;   --policy load\_balancing \\

&nbsp;   --no\_rt \\

&nbsp;   --no\_vis

done

On Windows/PowerShell you can adapt the loop or use the provided

run\_many\_seeds.ps1 script as a template.



The resulting logs can be parsed with your own scripts (or a notebook) to

recompute the metrics reported in the paper (mean service latency, ACK delay,

end-to-end latency, coverage).



If you just want the exact numbers used in the camera-ready, you can work

directly from the Excel files in REPRODUCIBILITY/:



Each poi\_XXXX\_all\_seeds.xlsx file contains per-PoI measurements for all

seeds at that workload.



final\_summary.xlsx aggregates per-configuration means and is what we used

to populate Tables 2–4 and to plot the latency–coverage Pareto fronts.



Adjusting scenarios

You can customize the scenario via:



config.py – Map size, mission horizon, communication radius, default

speeds, buffer sizes, etc.



CLI arguments in run\_simulation.py – Override many of the defaults for

specific runs (number of PoIs, K, ρ, buffer size, etc.).



Protocol files:



eqc\_protocol.py – leader logic (patrol pattern, assignment policy, timers)



vqc\_protocol.py – follower logic (navigation, satellite behavior,

delivery logic)



poi\_protocol.py – PoI behavior



These are the same components described in the Method section of the paper.



Reproducibility data

For convenience, the repository includes a minimal reproducibility package in

REPRODUCIBILITY/:



Raw, per-PoI measurements for each workload (P = 1000, 2500, 4000);



The aggregated summary used to build the tables and figures.



This means that you can fully reconstruct the plots and tables even if you

do not re-run the simulator, as long as you can read .xlsx files (e.g. with

Python/pandas, R, or any spreadsheet tool).



Citation

If you use this code or data in academic work, please cite the paper:



bibtex

Copiar código

@inproceedings{QuispeArias2025LeaderFollower,

&nbsp; author    = {Sebasti{\\'a}n Quispe Arias and

&nbsp;              Markus Endler and

&nbsp;              Bruno Jos{\\'e} Olivieri de Souza},

&nbsp; title     = {Leader-Follower Sizing in Heterogeneous UAV Swarms for SAR:

&nbsp;              Latency-Coverage Trade-Offs via Discrete-Event Simulation},

&nbsp; booktitle = {Proceedings of LAFUSION 2025},

&nbsp; year      = {2025},

&nbsp; note      = {Paper ID 23}

}

