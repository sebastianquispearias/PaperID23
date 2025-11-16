# ===== config.py (versión con escala) =====
import random
from typing import List, Dict, Tuple

# ---------- ESCALA ----------
SCALE = 1.0 #/ 24.0  # cada unidad del simulador representa 1/24 de metro real

# ---------- PARÁMETROS BASE (reales) ----------
L_BASE         = 1200.0     # m
R_CAMERA_BASE  = 80.7       # m
R_DETECT_BASE  = 5.0        # m
R_COMM_BASE    = 200.0  #30    # m
H_EQC_BASE     = 60.0       # m
H_VQC_BASE     = 40.0       # m
EQC_SPEED_BASE = 6.0  #6      # m/s
VQC_SPEED_BASE = 12.0  #12     # m/s
DURATION_BASE  = 2400       # s (no se escala el tiempo; escalamos velocidades/distancias)

# ---------- PARÁMETROS ESCALADOS (los que usa el simulador) ----------
L         = L_BASE * SCALE
R_CAMERA  = R_CAMERA_BASE * SCALE
R_DETECT  = R_DETECT_BASE * SCALE
R_COMM    = R_COMM_BASE * SCALE
h_eqc     = H_EQC_BASE * SCALE
h_vqc     = H_VQC_BASE * SCALE
EQC_SPEED = EQC_SPEED_BASE * SCALE
VQC_SPEED = VQC_SPEED_BASE * SCALE
DURATION  = DURATION_BASE   # mantenemos segundos “reales” (ya escalamos distance/speed)

# ---------- OTROS PARÁMETROS DE MISIÓN ----------
POIS: List[Dict] = []
ASSIGNMENT_POLICY = 'load_balancing'
M = 5
NUM_VQCS = 5
MAX_ASSIGN_PER_ENCOUNTER = 9999999999999999999999999999999999999999999

# ---------- Métricas globales ----------
METRICS = {
    "unique_ids": set(),
    "redundant": 0,
    "collected_but_not_delivered": 0,
    "global_score": 0.0,
    "eqc_reports": [],
    "lat_service_all": [],
    "lat_contact_all": [],
    "lat_e2e_all": [],
    "t_detect_all": [],
    "cam_raw_all": 0,
    "cam_hits_all": 0,
    "eqc_finished": 0,
}

BUMP_FREE_ON_ASSIGNED_DELIVER = False
USE_GLOBAL_COLLECTION_LOCK = True
WARN_XY_VS_3D = True
XY3D_MAX_SAMPLES = 5
COLLECTED_LABELS = set()
LEADER_OF: Dict[int, int] = {}

MAX_POIS = 100
URGENCY_WEIGHTS = {1: 0.2, 2: 0.5, 3: 1.0}

# ---------- POSICIONES / RUTAS (en BASE y luego se escalan) ----------
# Define los waypoints en COORDENADAS BASE (0..L_BASE y alturas H_*_BASE):
EQC_INIT_POS_BASE: Tuple[float, float, float] = (0.0, 0.0, H_EQC_BASE)

# Zig-zag horizontal/vertical en cuadrícula cada 120 m (puedes ajustarlo)
def _zigzag_rows(y_step=120.0):
    rows = []
    y = 0.0
    while y <= L_BASE:
        if int(y / y_step) % 2 == 0:
            rows += [(0.0, y, H_EQC_BASE), (L_BASE, y, H_EQC_BASE)]
        else:
            rows += [(L_BASE, y, H_EQC_BASE), (0.0, y, H_EQC_BASE)]
        y += y_step
    return rows

def _zigzag_cols(x_step=120.0):
    cols = []
    x = L_BASE
    while x >= 0.0:
        if int((L_BASE - x) / x_step) % 2 == 0:
            cols += [(x, 0.0, H_EQC_BASE), (x, L_BASE, H_EQC_BASE)]
        else:
            cols += [(x, L_BASE, H_EQC_BASE), (x, 0.0, H_EQC_BASE)]
        x -= x_step
    return cols

USE_MANUAL_WAYPOINTS = True
NUM_EQCS = 4

# Waypoints por líder en COORDENADAS BASE
EQC_WAYPOINTS_BASE: Dict[int, List[Tuple[float, float, float]]] = {
    0: _zigzag_rows(y_step=120.0),
    1: _zigzag_cols(x_step=120.0),
    2: list(reversed(_zigzag_rows(y_step=120.0))),  # mismo patrón invertido
    3: list(reversed(_zigzag_cols(x_step=120.0))),
}

# Helper para escalar una lista/dict de waypoints base -> simulador
def _scale_points(pts):
    return [(x * SCALE, y * SCALE, z * SCALE) for (x, y, z) in pts]

def _scale_wp_dict(wp_dict):
    out = {}
    for k, pts in wp_dict.items():
        out[k] = _scale_points(pts)
    return out

EQC_INIT_POS = tuple(v * SCALE for v in EQC_INIT_POS_BASE)
EQC_WAYPOINTS: Dict[int, List[Tuple[float, float, float]]] = _scale_wp_dict(EQC_WAYPOINTS_BASE)

# ---------- PoIs ----------
def get_pois(seed: int, n: int) -> List[Dict]:
    rng = random.Random(seed)
    base: List[Dict] = []
    for i in range(n):
        x = rng.uniform(0.0, L)
        y = rng.uniform(0.0, L)
        urg = rng.randint(1, 3)
        base.append({
            "id":    f"{seed:03d}-{i:03d}",
            "label": f"POI-{i+1}",
            "coord": (x, y),
            "urgency": urg
        })
    return base
