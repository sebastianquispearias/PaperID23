# experiments.py — barrido (K, rho) con MULTI-seed → un solo Excel
import subprocess, re, os, datetime, csv
import pandas as pd  # <- necesitas: pip install pandas openpyxl
import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--seed", type=int, help="run only this seed")
args, _ = parser.parse_known_args()

# ---------- Parámetros globales ----------
DEFAULT_SEEDS = [123,114,115,116,117,118,119,120,121,122]
SEEDS = DEFAULT_SEEDS[:]  # copia

parser = argparse.ArgumentParser()
parser.add_argument("--seed", type=int, help="run only this seed")
args, _ = parser.parse_known_args()


# 3) override por variable de entorno (si viene)
_env = os.getenv("EXP_SEEDS")
if _env:
    try:
        SEEDS = [int(x) for x in _env.split(",") if x.strip()]
    except ValueError:
        print(f"⚠️ EXP_SEEDS inválido: {_env}; usando DEFAULT_SEEDS")

# 4) override final por CLI (máxima prioridad)
if args.seed is not None:
    SEEDS = [args.seed]

print(f"→ SEEDS en uso: {SEEDS}")  # ayuda a verificar rápidamente

POIS              = 2500
BUFFER_M          = 5
R_CAM             = 84.9
EQC_SPEED_DEFAULT = 6.0
VQC_SPEED_DEFAULT = 12.0
POLICY            = "load_balancing"

# Barridos
K_LIST   = [1, 2, 3, 4]
RHO_LIST = [1, 2, 3, 4]

# ---------- Salidas ----------
STAMP  = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
OUTTAG = f"seed{SEEDS[0]}" if len(SEEDS) == 1 else None
OUTDIR = f"runs_{OUTTAG+'_' if OUTTAG else ''}{STAMP}"

os.makedirs(OUTDIR, exist_ok=True)
CSV_PATH = os.path.join(OUTDIR, "results_all.csv")
XLSX_PATH = os.path.join(OUTDIR, "results_all.xlsx")

RESULT_RE = re.compile(
    r"RESULT .*?"
    r"assigns_sent=(\d+)\s+assign_success=(\d+)\s+"
    r"redundant_delivers=(\d+)\s+"
    r"avg_latency=([\d\.]+)s\s+p95_latency=([\d\.]+)s\s+"
    r"ack_delay_mean=([\d\.]+)s\s+ack_delay_p95=([\d\.]+)s\s+"
    r"e2e_mean=([\d\.]+)s\s+e2e_p95=([\d\.]+)s\s+"
    r"coverage=(\d+)/(\d+)\s+coverage_rate=([\d\.]+)\s+"
    r"global_score=([\d\.]+)\s+"
    r"cam_raw=(\d+)\s+cam_matches=(\d+)",
    re.IGNORECASE
)

def run_case(seed, K, rho):
    num_vqcs = K * rho
    prefix   = f"seed{seed}_K{K}_rho{rho}_pois{POIS}_M{BUFFER_M}"
    fig_prefix_full = os.path.join(OUTDIR, prefix)
    log_path = os.path.join(OUTDIR, f"{prefix}.txt")

    cmd = (
        f"python run_simulation.py"
        f" --seed {seed}"
        f" --num_pois {POIS}"
        f" --num_vqcs {num_vqcs}"
        f" --buffer_size {BUFFER_M}"
        f" --eqc_speed {EQC_SPEED_DEFAULT}"
        f" --vqc_speed {VQC_SPEED_DEFAULT}"
        f" --camera_reach {R_CAM}"
        f" --policy {POLICY}"
        f" --num_eqcs {K}"
        f" --no_rt --no_vis"
        f" --fig_prefix \"{fig_prefix_full}\""
    )

    print(f"\n🏃 Ejecutando: {cmd}")
    proc = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    log  = proc.stdout + "\n" + proc.stderr

    with open(log_path, "w", encoding="utf-8") as f:
        f.write(log)

    base = {
        "seed": seed, "K": K, "rho": rho, "num_pois": POIS, "num_vqcs": num_vqcs, "M": BUFFER_M,
        "eqc_speed": EQC_SPEED_DEFAULT, "vqc_speed": VQC_SPEED_DEFAULT, "R_CAM": R_CAM, "policy": POLICY,
        "ok": int(proc.returncode == 0), "log_path": log_path
    }
    if proc.returncode != 0:
        print(f"❌ Error en {prefix} (ver {log_path})")
        return base

    m = RESULT_RE.search(log)
    if not m:
        print(f"⚠️ No encontré línea RESULT en {prefix}; revisa {log_path}")
        return base

    assigns_sent        = int(m.group(1))
    assign_success      = int(m.group(2))
    redundant_delivers  = int(m.group(3))
    avg_latency         = float(m.group(4))
    p95_latency         = float(m.group(5))
    ack_mean            = float(m.group(6))
    ack_p95             = float(m.group(7))
    e2e_mean            = float(m.group(8))
    e2e_p95             = float(m.group(9))
    coverage_num        = int(m.group(10))
    coverage_den        = int(m.group(11))
    coverage_rate       = float(m.group(12))
    global_score        = float(m.group(13))
    cam_raw             = int(m.group(14))
    cam_matches         = int(m.group(15))
    assign_rate         = (assign_success / assigns_sent) if assigns_sent > 0 else None

    base.update({
        "assign_success": assign_success,
        "assigns_sent": assigns_sent,
        "assign_rate": assign_rate,
        "redundant_delivers": redundant_delivers,
        "avg_latency_s": avg_latency,
        "p95_latency_s": p95_latency,
        "ack_mean_s": ack_mean,
        "ack_p95_s": ack_p95,
        "e2e_mean_s": e2e_mean,
        "e2e_p95_s": e2e_p95,
        "coverage": f"{coverage_num}/{coverage_den}",
        "coverage_rate": coverage_rate,
        "global_score": global_score,
        "cam_raw": cam_raw,
        "cam_matches": cam_matches,
    })
    return base

# ---------- Ejecuta TODO y escribe CSV + Excel ----------
rows = []
header = [
    "seed","K","rho","num_pois","num_vqcs","M",
    "eqc_speed","vqc_speed","R_CAM","policy",
    "assign_success","assigns_sent","assign_rate",
    "redundant_delivers",
    "avg_latency_s","p95_latency_s",
    "ack_mean_s","ack_p95_s",
    "e2e_mean_s","e2e_p95_s",
    "coverage","coverage_rate",
    "global_score","cam_raw","cam_matches",
    "ok","log_path"
]

for seed in SEEDS:
    for K in K_LIST:
        for rho in RHO_LIST:
            res = run_case(seed, K, rho)
            # Garantiza todas las columnas:
            row = [res.get(col, "") for col in header]
            rows.append(row)

# CSV
with open(CSV_PATH, "w", newline="", encoding="utf-8") as f:
    writer = csv.writer(f)
    writer.writerow(header)
    writer.writerows(rows)

# Excel (mismas columnas, una sola hoja)
df = pd.DataFrame(rows, columns=header)
with pd.ExcelWriter(XLSX_PATH, engine="openpyxl") as xw:
    df.to_excel(xw, sheet_name="results", index=False)

print(f"\n✅ Terminado.\n📄 CSV:   {CSV_PATH}\n📊 Excel: {XLSX_PATH}\n📁 Carpeta: {OUTDIR}")
