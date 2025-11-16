"""
run_simulation.py
Main script to set up and run the simulation:
- Configures communication, timer, mobility, and visualization handlers.
- Initializes E-QC, V-QCs, and PoI nodes.
- Starts the simulation.
"""

#python run_simulation.py --seed 123 --num_pois 1000 --num_eqcs 2 --num_vqcs 4 --buffer_size 5 --eqc_speed 6.0 --vqc_speed 12.0 --camera_reach 84.9 --policy load_balancing --no_rt  --debug --no_vis

import logging
import random
import argparse                                       
import config     
import sys
import os
# === [NEW] plotting & data ===
# === [NEW] Matplotlib backend para headless ===
import math 
import matplotlib
matplotlib.use("Agg")

import matplotlib.pyplot as plt
from collections import defaultdict

from gradysim.simulator.handler.communication import CommunicationHandler, CommunicationMedium
from gradysim.simulator.handler.timer import TimerHandler
from gradysim.simulator.handler.mobility import MobilityHandler, MobilityConfiguration
from gradysim.simulator.simulation import SimulationBuilder, SimulationConfiguration
from gradysim.simulator.handler.visualization import VisualizationHandler, VisualizationConfiguration

from poi_protocol import POIProtocol
from eqc_protocol import EQCProtocol
from vqc_protocol import VQCProtocol
from config import EQC_INIT_POS
import config

def emit_run_summary(root, log_fname, args, leaders_used, mobility_speed):
    import math
    import config

    def _mean(v): return (sum(v)/len(v)) if v else float('nan')
    def _p95(v):
        if not v: return float('nan')
        vs = sorted(v)
        return vs[int(0.95 * (len(vs) - 1))]

    # Parámetros de la corrida
    K            = int(leaders_used or 0)
    NVQC         = int(getattr(config, "NUM_VQCS", 0))
    M            = int(getattr(config, "M", 0))
    policy       = getattr(config, "ASSIGNMENT_POLICY", "greedy")
    speed        = float(mobility_speed)
    rcam         = float(getattr(config, "R_CAMERA", 0.0))
    seed         = int(getattr(args, "seed", -1))
    area_L       = float(getattr(config, "L", 0.0))
    num_pois     = len(getattr(config, "POIS", []))

    # Métricas globales
    uniq         = len(config.METRICS.get("unique_ids", set()))
    redundant    = int(config.METRICS.get("redundant", 0))
    score        = float(config.METRICS.get("global_score", 0.0))

    eqc_reports  = list(config.METRICS.get("eqc_reports", []))
    assigns_tot  = sum(int(r.get("assigns", 0)) for r in eqc_reports)
    success_tot  = sum(int(r.get("success", 0)) for r in eqc_reports)
    success_rate = (success_tot/assigns_tot) if assigns_tot>0 else float('nan')

    Ls_all   = list(config.METRICS.get("lat_service_all", []))
    Lc_all   = list(config.METRICS.get("lat_contact_all", []))
    Le2e_all = list(config.METRICS.get("lat_e2e_all", []))
    Td_all   = list(config.METRICS.get("t_detect_all", []))

    cam_raw_all  = int(config.METRICS.get("cam_raw_all", 0))
    cam_hits_all = int(config.METRICS.get("cam_hits_all", 0))

    # Derivadas útiles
    coverage_rate   = (uniq/num_pois) if num_pois>0 else float('nan')
    redundancy_rate = (redundant/max(uniq,1)) if uniq>0 else float('nan')

    # Estadísticos globales con muestras crudas
    Ls_mean,  Ls_p95  = _mean(Ls_all),  _p95(Ls_all)
    Lc_mean,  Lc_p95  = _mean(Lc_all),  _p95(Lc_all)
    Le_mean,  Le_p95  = _mean(Le2e_all), _p95(Le2e_all)
    Td_mean,  Td_p95  = _mean(Td_all),  _p95(Td_all)

    # ===== TABLA (markdown) =====
    lines = []
    lines.append("")
    lines.append("# 📊 Resumen de la corrida")
    lines.append("")
    lines.append("| Parámetro / Métrica | Valor |")
    lines.append("|---|---:|")
    lines.append(f"| Líderes (K) | {K} |")
    lines.append(f"| VQCs (N) | {NVQC} |")
    lines.append(f"| Ratio N/K | {NVQC/(K or 1):.2f} |")
    lines.append(f"| Buffer M | {M} |")
    lines.append(f"| Política | {policy} |")
    lines.append(f"| Velocidad (u/s) | {speed:.1f} |")
    lines.append(f"| R_CAM (u) | {rcam:.1f} |")
    lines.append(f"| Semilla | {seed} |")
    lines.append(f"| Área | {area_L}×{area_L} u |")
    lines.append(f"| PoIs totales | {num_pois} |")
    lines.append(f"| Assigns totales | {assigns_tot} |")
    lines.append(f"| Successful delivers | {success_tot} |")
    lines.append(f"| Success rate | {success_rate:.2f} |")
    lines.append(f"| PoIs únicos (coverage) | {uniq} ({coverage_rate*100:.1f}%) |")
    lines.append(f"| Redundancias | {redundant} (ratio≈{redundancy_rate:.2f}) |")
    lines.append(f"| Puntuación ponderada | {score:.2f} |")
    lines.append(f"| Cámara detecciones (raw) | {cam_raw_all} |")
    lines.append(f"| Cámara match con PoIs | {cam_hits_all} |")
    lines.append(f"| Service latency μ / p95 (s) | {Ls_mean:.3f} / {Ls_p95:.3f} |")
    lines.append(f"| Reporting–ACK delay μ / p95 (s) | {Lc_mean:.3f} / {Lc_p95:.3f} |")
    lines.append(f"| End-to-end μ / p95 (s) | {Le_mean:.3f} / {Le_p95:.3f} |")
    lines.append(f"| Time-to-detect μ / p95 (s) | {Td_mean:.3f} / {Td_p95:.3f} |")

    # ===== SUBTABLA: detalle por EQC =====
    lines.append("")
    lines.append("### 📦 Detalle por EQC (leader)")
    lines.append("")
    lines.append("| EQC | Assigns | Success | Rate |")
    lines.append("|---:|---:|---:|---:|")
    for rep in sorted(eqc_reports, key=lambda r: r.get("eqc_id", -1)):
        a = int(rep.get("assigns", 0))
        s = int(rep.get("success", 0))
        rate = (s/a) if a>0 else float('nan')
        lines.append(f"| {rep.get('eqc_id','?')} | {a} | {s} | {rate:.2f} |")

    # ===== ANÁLISIS BREVE =====
    lines.append("")
    lines.append("## 🧠 Análisis breve")
    lines.append(f"- Cobertura: {uniq}/{num_pois} ({coverage_rate*100:.1f}%).")
    lines.append(f"- Éxito de assigns: {success_tot}/{assigns_tot} (rate {success_rate:.2f}).")
    lines.append(f"- Redundancia observada: {redundant} (≈{redundancy_rate:.2f} por PoI único).")
    lines.append(f"- Latencias: service p95={Ls_p95:.2f}s, contacto p95={Lc_p95:.2f}s, e2e p95={Le_p95:.2f}s.")
    lines.append("  - Si `contact_overhead` p95 ≪ `service` p95, el cuello es navegación/servicio;")
    lines.append("    si no, puede faltar frescura de huecos o hay colas de entrega.")

    # ===== GLOSARIO =====
    lines.append("")
    lines.append("## 📖 Glosario")
    lines.append("- **Assigns totales / Successful delivers**: asignaciones desde EQC / entregas recibidas por EQC.")
    lines.append("- **Success rate**: successful_delivers / assigns_totales.")
    lines.append("- **PoIs únicos (coverage)**: PoIs distintos reportados (sin duplicados).")
    lines.append("- **Redundancias**: reportes de PoIs ya colectados previamente.")
    lines.append("- **Puntuación ponderada**: suma de pesos por urgencia de PoIs entregados (w₃≥w₂≥w₁).")
    lines.append("- **Service latency**: `t_arrive − t_detect`. Del avistamiento del EQC hasta llegada del VQC.")
    lines.append("- **Reporting–ACK delay**: `t_deliver_ack − t_arrive`. Tiempo de contacto/confirmación.")
    lines.append("- **End-to-end**: `t_deliver_ack − t_spawn` (aprox. desde inicio de corrida).")
    lines.append("- **Time-to-detect**: `t_detect − t_spawn` (tiempo hasta que el EQC ve por 1ª vez el PoI).")
    lines.append("- **μ / p95**: media y percentil 95% de cada métrica.")

    # Imprimir en log y guardar .summary.md
    block = "\n".join(lines)
    for ln in lines: root.info(ln)

    summary_fname = log_fname.replace(".txt", ".summary.md")
    try:
        with open(summary_fname, "w", encoding="utf-8") as f:
            f.write(block + "\n")
        root.info(f"📝 Summary written to: {summary_fname}")
    except Exception as e:
        root.warning(f"⚠️ Could not write summary file: {e}")

    # ===== Línea plana para parsers (experiments.py) =====
    # Nota: "assign_success" = successful_delivers (suma global),
    #       "redundant_delivers" = redundant
    try:
        rho_val = NVQC / max(1, K)
    except Exception:
        rho_val = float('nan')
    # Línea corta para parsers antiguos (experiments.py)
    root.info(f"Assigns sent={assigns_tot}, successful delivers={success_tot} (rate={success_rate:.2f})")
    root.info(f"Global mission score = {score:.2f}")
    root.info(f"Cámara hizo {cam_raw_all} detecciones en total, {cam_hits_all} coincidencias")

    result_line = (
        f"RESULT seed={seed} "
        f"K={K} rho={rho_val:.2f} num_pois={num_pois} num_vqcs={NVQC} "
        f"M={M} policy={policy} speed={speed:.3f} R_CAMERA={rcam:.1f} "
        f"assigns_sent={assigns_tot} assign_success={success_tot} "
        f"redundant_delivers={redundant} "
        f"avg_latency={Ls_mean:.4f}s p95_latency={Ls_p95:.4f}s "
        f"ack_delay_mean={Lc_mean:.4f}s ack_delay_p95={Lc_p95:.4f}s "
        f"e2e_mean={Le_mean:.4f}s e2e_p95={Le_p95:.4f}s "
        f"coverage={uniq}/{num_pois} coverage_rate={coverage_rate:.4f} "
        f"global_score={score:.4f} "
        f"cam_raw={cam_raw_all} cam_matches={cam_hits_all}"
    )
    root.info(result_line)


def render_trajectory_figures(positions, poi_positions, L, K, rho, seed, prefix, leader_of):
    """
    Genera:
      1) {prefix}_ALL_K{K}_rho{rho:.2f}_seed{seed}.png  (todas las trayectorias)
      2) {prefix}_EQC{e}_K{K}_rho{rho:.2f}_seed{seed}.png  (una por EQC e + sus VQCs)
    """
    by_agent = defaultdict(list)
    roles = {}
    for row in positions:
        by_agent[row["agent"]].append(row)
        roles[row["agent"]] = row["role"]

    # --- Figura 1: todo junto ---
    fig, ax = plt.subplots(figsize=(8, 8))
    # PoIs estáticos
    if poi_positions:
        xs = [p["x"] for p in poi_positions]
        ys = [p["y"] for p in poi_positions]
        ax.scatter(xs, ys, marker='x', s=25, label='PoIs', linewidths=1)

    # Trayectorias
    for agent, series in by_agent.items():
        series = sorted(series, key=lambda r: r["timestamp"])
        xs = [r["x"] for r in series]
        ys = [r["y"] for r in series]
        r  = roles.get(agent, "?")
        if r == "eqc":
            ax.plot(xs, ys, '-',  linewidth=2, label=f"EQC-{agent}")
        else:
            ax.plot(xs, ys, '-',  linewidth=0.8, alpha=0.6, label=f"VQC-{agent}")

    ax.set_xlim(0, L); ax.set_ylim(0, L)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
    ax.set_title(f'Trajectories — K={K}, rho={rho:.2f}, seed={seed}')
    # Leyenda sin duplicados
    handles, labels = ax.get_legend_handles_labels()
    uniq = dict(zip(labels, handles))
    ax.legend(uniq.values(), uniq.keys(), loc='upper right', fontsize=8)
    fname_all = f"{prefix}_ALL_K{K}_rho{rho:.2f}_seed{seed}.png"
    plt.tight_layout()
    plt.savefig(fname_all, dpi=200)
    plt.close(fig)

    # --- Figuras por líder: EQC e + sus VQCs (según leader_of) ---
    followers_by_leader = defaultdict(list)
    for v_id, e_id in leader_of.items():
        followers_by_leader[e_id].append(v_id)

    for e_id, v_list in followers_by_leader.items():
        fig, ax = plt.subplots(figsize=(8, 8))
        if poi_positions:
            xs = [p["x"] for p in poi_positions]
            ys = [p["y"] for p in poi_positions]
            ax.scatter(xs, ys, marker='x', s=25, label='PoIs', linewidths=1)

        # EQC e_id
        if e_id in by_agent:
            series = sorted(by_agent[e_id], key=lambda r: r["timestamp"])
            xs = [r["x"] for r in series]; ys = [r["y"] for r in series]
            ax.plot(xs, ys, '-', linewidth=2.4, label=f"EQC-{e_id}")

        # Sus VQCs
        for vid in v_list:
            if vid not in by_agent:
                continue
            series = sorted(by_agent[vid], key=lambda r: r["timestamp"])
            xs = [r["x"] for r in series]; ys = [r["y"] for r in series]
            ax.plot(xs, ys, '-', linewidth=1.0, alpha=0.8, label=f"VQC-{vid}")

        ax.set_xlim(0, L); ax.set_ylim(0, L)
        ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
        ax.set_title(f'Leader EQC-{e_id} and followers — K={K}, rho={rho:.2f}, seed={seed}')
        handles, labels = ax.get_legend_handles_labels()
        uniq = dict(zip(labels, handles))
        ax.legend(uniq.values(), uniq.keys(), loc='upper right', fontsize=8)
        fname = f"{prefix}_EQC{e_id}_K{K}_rho{rho:.2f}_seed{seed}.png"
        plt.tight_layout()
        plt.savefig(fname, dpi=200)
        plt.close(fig)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Ejecuta simulaciones con parámetros variables")
    # DESPUÉS (sin choices, acepta 6.0 y 60.0, etc.)
    parser.add_argument('--num_pois',      type=int,   required=True, help='Cantidad de PoIs a usar')
    parser.add_argument('--num_vqcs',      type=int,   required=True, help='Número de V-QCs')
    parser.add_argument('--buffer_size',   type=int,   required=True, help='Tamaño máximo de buffer M')
    parser.add_argument('--eqc_speed',  type=float, default=None,
        help='Velocidad de los líderes (EQC) en m/s; si no se pasa, se usa config.EQC_SPEED_BASE')
    parser.add_argument('--vqc_speed',  type=float, default=None,
        help='Velocidad de los seguidores (VQC) en m/s; si no se pasa, se usa config.VQC_SPEED_BASE')


    parser.add_argument('--camera_reach',  type=float, required=True, help='Alcance oblicuo de la cámara (R_CAMERA)')

    
    parser.add_argument('--seed',          type=int,required=True,help='Semilla para generar PoIs y posiciones iniciales')
    parser.add_argument('--policy',        choices=['greedy','round_robin','load_balancing'], default='greedy', help='Política de asignación: greedy | round_robin | load_balancing')
    parser.add_argument('--num_eqcs', type=int, default=None,
        help='Nº de EQCs (líderes) a instanciar cuando USE_MANUAL_WAYPOINTS=True. '
            'Si no se pasa, se usan todas las rutas definidas.')
    parser.add_argument('--save_figs', action='store_true',
        help='Si se activa, corre la simulación paso a paso y guarda trayectorias e imágenes.')
    parser.add_argument('--figdir', type=str, default='figs',
        help='Carpeta de salida para imágenes y CSV (por defecto: figs/).')
    parser.add_argument('--fig_prefix', type=str, default='traj',
        help='Prefijo base para los nombres de los PNGs (por defecto: "traj").')

    # Control fino de tiempo real y debug:
    parser.add_argument('--no_rt', action='store_true',
        help='Fuerza real_time=False (útil para generar figuras rápido).')
    parser.add_argument('--debug', action='store_true',
        help='Activa debug=True en SimulationConfiguration.')
    parser.add_argument('--no_vis', action='store_true',
        help='No registra VisualizationHandler (más rápido para corridas masivas).')

    args = parser.parse_args()
    random.seed(args.seed)  
    config.POIS = config.get_pois(seed=args.seed, n=args.num_pois)
    config.NUM_VQCS   = args.num_vqcs    
    config.M          = args.buffer_size 
    config.R_CAMERA   = args.camera_reach * config.SCALE   # antes: = args.camera_reach
        # Overrides de velocidades por CLI (si vienen)
    if args.eqc_speed is not None:
        config.EQC_SPEED = args.eqc_speed * config.SCALE
    if args.vqc_speed is not None:
        config.VQC_SPEED = args.vqc_speed * config.SCALE

    #mobility_speed    = args.speed * config.SCALE          # antes: = args.speed
    # Default del mundo: hereda la velocidad efectiva de los EQC
    mobility_speed = float(config.EQC_SPEED)


    config.ASSIGNMENT_POLICY = args.policy
    # --- Reset de estado global para esta corrida (antes de crear ningún nodo) ---
    config.METRICS = {
        "unique_ids": set(),
        "redundant": 0,
        "global_score": 0,
        # acumuladores globales que EQC.finish() va extendiendo/sumando
        "lat_service_all": [],
        "lat_contact_all": [],
        "lat_e2e_all": [],
        "t_detect_all": [],
        "cam_raw_all": 0,
        "cam_hits_all": 0,
        "eqc_reports": [],
        "eqc_finished": 0,
    }
    if getattr(config, "USE_GLOBAL_COLLECTION_LOCK", False):
        config.COLLECTED_LABELS = set()


    # === LOGGING (usa fig_prefix como base del .txt y crea la carpeta si hace falta) ===
    log_base = args.fig_prefix            # puede incluir subcarpetas
    log_dir = os.path.dirname(log_base)
    if log_dir:
        os.makedirs(log_dir, exist_ok=True)
    log_fname = f"{log_base}.txt"

    root = logging.getLogger()
    root.setLevel(logging.DEBUG if args.debug else logging.INFO)

    fmt = logging.Formatter("%(asctime)s %(name)-12s %(levelname)-8s %(message)s")
    ch = logging.StreamHandler(); ch.setFormatter(fmt); root.addHandler(ch)
    fh = logging.FileHandler(log_fname, mode="w", encoding="utf-8")
    fh.setFormatter(fmt)
    root.addHandler(fh)
    root.info(f"✅ Logging to file: {log_fname}")


    root.info(
        f"✅ Simulation start — seed={args.seed}, num_pois={len(config.POIS)}, "
        f"duration={config.DURATION}s, VQCs={config.NUM_VQCS}, area={config.L}×{config.L}, "
        f"speed(default)={mobility_speed} m/s, "
        f"vEQC={config.EQC_SPEED} m/s, vVQC={config.VQC_SPEED} m/s, "
        f"speed(default)={mobility_speed} m/s, camera_reach={config.R_CAMERA}"
        
    )


 #####################——— Construcción de la simulación ———
    # Si vamos a guardar figuras, conviene paso a paso y sin real_time para ir rápido
    real_time_flag = True
    if args.save_figs or args.no_rt:
        real_time_flag = False

    sim_cfg = SimulationConfiguration(
        duration=config.DURATION,
        debug=args.debug,              # << ahora toma el flag --debug
        real_time=real_time_flag       # << respeta --save_figs / --no_rt
    )
    builder = SimulationBuilder(sim_cfg)

    # [NEW] guardamos IDs para graficar
    eqc_ids = []
    vqc_ids = []
    poi_ids = []

    use_manual = getattr(config, "USE_MANUAL_WAYPOINTS", False)

    if use_manual:
        all_routes = config.EQC_WAYPOINTS
        # Si no pasas --num_eqcs, usa todas las rutas definidas
        E = args.num_eqcs if args.num_eqcs is not None else len(all_routes)
        # Limita a [1, len(all_routes)] para evitar índices inválidos
        E = max(1, min(E, len(all_routes)))
    else:
        # Si generas rutas dinámicas y no pasas --num_eqcs, por defecto 1 líder
        E = args.num_eqcs if args.num_eqcs is not None else 1
        E = max(1, E)

    # --- Añade SOLO esos E EQCs (¡no añadas un EQC suelto en EQC_INIT_POS!) ---
    for e in range(E):
        start = tuple(config.EQC_WAYPOINTS[e][0])   # primer waypoint de cada ruta
        eid = builder.add_node(EQCProtocol, start)  # [NEW]
        eqc_ids.append(eid)                         # [NEW]
        root.info(f"➕ EQC-{e} @ {start}")


    # --- Construye el mapeo VQC → líder (una sola vez) ---
    config.LEADER_OF = {}
    offset = E  # los VQCs van después de los EQCs en el orden de creación
    for i in range(config.NUM_VQCS):
        vid_runtime = offset + i
        leader_id   = i % E                         # reparto uniforme (0..E-1, 0..E-1, …)
        config.LEADER_OF[vid_runtime] = leader_id
    root.info(f"👥 LEADER_OF: {config.LEADER_OF}")

    for i in range(config.NUM_VQCS):
        # Asignación de líder ya hecha antes: leader_id = i % E (ver LEADER_OF)
        leader_idx = i % E

        # Primer waypoint del líder: donde arranca ese EQC
        leader_start = tuple(config.EQC_WAYPOINTS[leader_idx][0])  # (x, y, z)  ✔
        # Pequeño offset satélite para no colisionar y asegurar contacto
        r_init = min(20.0, 0.1 * config.R_COMM)   # 20 u o el 10% del radio de comunicación
        ang = 2.0 * math.pi * (i % 8) / 8.0       # distribuye 8 posiciones alrededor
        dx = r_init * math.cos(ang)
        dy = r_init * math.sin(ang)

        spawn_pos = (leader_start[0] + dx, leader_start[1] + dy, config.h_vqc)
        vid = builder.add_node(VQCProtocol, spawn_pos)
        vqc_ids.append(vid)
        root.info(f"➕ VQC-{i} (ldr={leader_idx}) @ {spawn_pos}")


    if not hasattr(config, "POI_LABEL2NODE"):   config.POI_LABEL2NODE = {}
    if not hasattr(config, "POI_ID2NODE"):      config.POI_ID2NODE    = {}
    if not hasattr(config, "POI_LABEL2COORD"):  config.POI_LABEL2COORD= {}
    if not hasattr(config, "POI_ID2LABEL"):     config.POI_ID2LABEL   = {}

    poi_ids = []  # ya lo tienes
    for poi in config.POIS:
        pid = builder.add_node(POIProtocol, (poi["coord"][0], poi["coord"][1], 0.0))
        poi_ids.append(pid)

        # Registra mapeos
        label = poi["label"]   # p.ej. "POI-95"
        pid_s = str(poi["id"]) # si tu 'id' es numérico, lo conservamos como str y como int
        config.POI_LABEL2NODE[label] = pid
        config.POI_ID2NODE[poi["id"]] = pid
        config.POI_LABEL2COORD[label] = tuple(poi["coord"])
        config.POI_ID2LABEL[poi["id"]] = label

    root.info(f"➕ Added {len(config.POIS)} POIProtocol nodes")


 # ——— Handler
    medium = CommunicationMedium(transmission_range=config.R_COMM)
    builder.add_handler(CommunicationHandler(medium))
    builder.add_handler(TimerHandler())
    builder.add_handler(MobilityHandler(MobilityConfiguration(default_speed=mobility_speed)))

    # Visualization solo si no lo desactivan y/o si realmente vamos a pintar
    # === Visualización con rangos alineados al escenario (0..L) y alturas ~40/60 u ===
    if not args.no_vis:
        z_top = max(getattr(config, "h_eqc", 60.0), getattr(config, "h_vqc", 40.0)) + 500.0
        viz_cfg = VisualizationConfiguration(
            x_range=(0.0, float(config.L)),
            y_range=(0.0, float(config.L)),
            z_range=(0.0, float(z_top)),
            open_browser=False,   # pon True si quieres abrir el visor web al arrancar
            update_rate=0.05      # tasa de refresco del visor
        )
        builder.add_handler(VisualizationHandler(viz_cfg))

    root.info("🔧 Handlers added")
 # ——— Ejecución ———
    sim = builder.build()
    root.info("▶️ Starting simulation")
    # Crear carpeta de salida si vamos a guardar figuras
    if args.save_figs:
        os.makedirs(args.figdir, exist_ok=True)

    if not args.save_figs:
        # Camino original (rápido)
        sim.start_simulation()
        root.info("🏁 Simulation complete")
    else:
        # [NEW] Camino “paso a paso” para recolectar trayectorias
        positions = []         # filas: {role, agent, timestamp, x, y, z}
        poi_positions = []     # filas: {x, y, z}

        # PoIs (estáticos)
        for pid in poi_ids:
            px, py, pz = sim.get_node(pid).position
            poi_positions.append({"x": px, "y": py, "z": pz})

        # Loop de simulación paso a paso
        step_count = 0
        while sim.step_simulation():
            # En Gradysim actual, el timestamp está en esta propiedad interna
            t = getattr(sim, "_current_timestamp", step_count)  # fallback por si cambia
            # EQCs
            for eid in eqc_ids:
                ex, ey, ez = sim.get_node(eid).position
                positions.append({"role": "eqc", "agent": eid, "timestamp": t, "x": ex, "y": ey, "z": ez})
            # VQCs
            for vid in vqc_ids:
                vx, vy, vz = sim.get_node(vid).position
                positions.append({"role": "vqc", "agent": vid, "timestamp": t, "x": vx, "y": vy, "z": vz})
            step_count += 1

        root.info(f"🖼️ Positions collected: {len(positions)} samples")

        # [NEW] Render de figuras
        try:
            render_trajectory_figures(
                positions=positions,
                poi_positions=poi_positions,
                L=config.L,
                K=len(eqc_ids),
                rho=(len(vqc_ids) / max(1, len(eqc_ids))),
                seed=args.seed,
                prefix=os.path.join(args.figdir, args.fig_prefix),
                leader_of=config.LEADER_OF
            )
            root.info("🖼️ Figures written successfully.")
        except Exception as e:
            root.warning(f"⚠️ Could not render figures: {e}")

    root.info("🏁 Simulation complete")
    emit_run_summary(root, log_fname, args, E, mobility_speed)
