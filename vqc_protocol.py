"""
Visiting Quadcopter (V-QC) protocol:
- Initial random roaming.
- Receives ASSIGN and visits PoIs.
- Locally detects PoI IDs and delivers them back.
"""

import json
import math
import logging
from typing import List, Tuple, Dict

from gradysim.protocol.interface import IProtocol
from gradysim.protocol.messages.communication import CommunicationCommand, CommunicationCommandType
from gradysim.protocol.messages.telemetry import Telemetry
from gradysim.protocol.plugin.random_mobility import RandomMobilityPlugin, RandomMobilityConfig
from gradysim.protocol.plugin.mission_mobility import MissionMobilityPlugin, MissionMobilityConfiguration, LoopMission

import config
from config import EQC_INIT_POS
from scipy.spatial.distance import euclidean

class VQCProtocol(IProtocol):
    def initialize(self) -> None:
        self.id = self.provider.get_id()
        self.log = logging.getLogger(f"VQC-{self.id}")

        self.pos = (0.0, 0.0, config.h_vqc)

        self.leader_id = getattr(config, "LEADER_OF", {}).get(self.id, 0)
        self.log.debug(f"Líder de VQC-{self.id}: EQC-{self.leader_id}")
        # >>> BEGIN PATCH: rank local de formación por líder
        try:
            followers = sorted([vid for vid, lid in getattr(config, "LEADER_OF", {}).items()
                                if lid == self.leader_id])
            self.formation_rank = followers.index(self.id) if self.id in followers else 0
        except Exception:
            # Fallback robusto si LEADER_OF no está completo; no rompe nada
            self.formation_rank = 0
        self.log.info(f"VQC-{self.id} → líder EQC-{self.leader_id} | rank local={self.formation_rank}")
        # >>> END PATCH

        self.next2visit: List[Tuple[Tuple[float, float, float], int]] = []
        #### [LATENCY] arrival timestamps (label -> t_arrive)
        self.arrival_ts = {}
        #### [/LATENCY]
        self.xy3d_warns = 0          # cuántas veces XY<=R pero 3D>R
        self.xy3d_samples = []       # hasta N ejemplos de la discrepancia
        self.discovered: List[Dict[str,str]] = []
        self.visited: List[str] = []
        # índice rápido coord->label para no recorrer todos los POIs cada vez
        # índice rápido (coord, urgency) -> label  para resolver etiquetas sin O(N)
        self.coordurg2label = {
            (p["coord"], p["urgency"]): p["label"]
            for p in config.POIS
        }
        self.coordurg2id = {
            (p["coord"], p["urgency"]): p["id"]
            for p in config.POIS
        }

        self.delivering = False
        self.state = "satellite"   

        # Siembra la posición real del líder (primer waypoint), en vez de EQC_INIT_POS
        try:
            leader_start = tuple(config.EQC_WAYPOINTS[self.leader_id][0])
        except Exception:
            leader_start = EQC_INIT_POS  # fallback seguro

        self.last_assign = {
            "eqc_pos":  leader_start,
            "eqc_time": self.provider.current_time()
        }
        self.log.info(f"🛰️ Posición inicial del líder sembrada para VQC-{self.id}: {self.last_assign['eqc_pos']}")


        self.log.info(f"🛫 VQC-{self.id} initialized at {self.pos}")

        #self.random = RandomMobilityPlugin(self, RandomMobilityConfig(x_range=(0,config.L), y_range=(0,config.L), z_range=(4.0,4.0), tolerance=1))
        self.mission = MissionMobilityPlugin(
            self, MissionMobilityConfiguration(speed=config.VQC_SPEED, loop_mission=LoopMission.NO, tolerance=1)
        )

        # Inicialmente arrancas en modo satélite a w₀
        self.maintain_satellite_mode()
        
        self.free = config.M - len(self.next2visit)
        self._viz_push()
        self.log.info("Modo satélite iniciado")
        t0 = self.provider.current_time()
        self.provider.schedule_timer("hello", t0+1)
        self.provider.schedule_timer("check_roam", t0+1)
        # === [LOG] Resumen inicial del VQC ===
        try:
            self.log.info(
                f"[VQC-{self.id}] init: state={self.state}, "
                f"visited={len(self.visited)}, discovered={len(self.discovered)}, "
                f"next2visit={len(self.next2visit)}, free={self.free}"
            )
        except Exception as e:
            self.log.warning(f"[VQC-{self.id}] No se pudo loguear init: {e}")
        # =====================================

        # Métricas de descubrimiento
        self.disc_casual   = 0   # fuera de misión
        self.disc_assigned = 0   # dentro de misión dirigida
        # Flags ejecución
        self._exec = {
            "handle_telemetry": False,
            "handle_timer.hello": False,
            "handle_packet.ASSIGN": False,
            "handle_packet.HELLO_ACK": False,
            "handle_packet.DELIVER_ACK": False,
        }
        

    # === [VISUALIZACIÓN] Publicar variables al panel ===
    def _viz_push(self) -> None:
        try:
            tv = self.provider.tracked_variables

            # Identidad y líder
            tv["vqc_id"]    = int(self.id) if hasattr(self, "id") else None
            tv["leader_id"] = int(self.leader_id) if hasattr(self, "leader_id") else None

            # Cola y capacidad
            try:
                raw_queue = list(getattr(self, "next2visit", []))  # [(coord3d, urg), ...]
                # Tamaño real de la cola (antes del recorte)
                tv["queue_size"] = len(raw_queue)

                # Mapea (coord3d, urg) -> label legible usando el índice (coord, urgency)
                queue_labels = [
                    self.coordurg2label.get(((c[0], c[1]), urg), str((c, urg)))
                    for (c, urg) in raw_queue
                ]
            except Exception:
                queue_labels = []
                tv["queue_size"] = 0

            # Recorta para no saturar la UI
            queue_labels = queue_labels[:100]
            tv["queue"]       = queue_labels
            tv["queue_shown"] = len(queue_labels)  # opcional: cuántos se muestran

            # Capacidad libre reportada por el VQC
            tv["free"] = int(getattr(self, "free", 0))

            # Visitas realizadas
            visited = list(getattr(self, "visited", []))
            tv["visited_cnt"]    = len(visited)
            tv["visited_labels"] = visited[:100]

            # Estado y tiempos
            tv["state"]             = getattr(self, "state", None)
            tv["last_assign_time"]  = getattr(self, "last_assign_time", None)
            tv["last_deliver_time"] = getattr(self, "last_deliver_time", None)

        except Exception as e:
            self.log.debug(f"[viz] VQC publish error: {e}")

    def predict_eqc_position(self, t: float) -> Tuple[float, float, float]:
        """
        Predice la posición del EQC a t segundos desde el inicio de la simulación,
        interpolando linealmente entre waypoints.
        """
        waypoints = config.EQC_WAYPOINTS[self.leader_id]

        v_eqc = config.EQC_SPEED

        # calcular duración de cada tramo
        durations = [max(1e-9, euclidean(a, b) / v_eqc)
                    for a, b in zip(waypoints, waypoints[1:])]
        

        total = sum(durations)

        if t <= 0:
            return waypoints[0]
        if t >= total:
            return waypoints[-1]

        elapsed = 0.0
        for (a, b), dur in zip(zip(waypoints, waypoints[1:]), durations):
            if elapsed + dur >= t:
                frac = (t - elapsed) / dur
                return (
                    a[0] + frac * (b[0] - a[0]),
                    a[1] + frac * (b[1] - a[1]),
                    a[2] + frac * (b[2] - a[2]),
                )
            elapsed += dur

        return waypoints[-1]
    # --- 2) Método auxiliar: calcular punto de intercepción predictiva ---
    def compute_intercept(self) -> Tuple[float, float, float]:
        """
        Itera para encontrar Δt tal que el VQC llegue justo donde estará el EQC.
        """
        now = self.provider.current_time()
        pos_vqc = self.pos                         # usa tu posición interna
        v_vqc = config.VQC_SPEED

        # estimación inicial: EQC en t = now
        pred = self.predict_eqc_position(now)
        dt = euclidean(pos_vqc, pred) / v_vqc

        # refinar con 5 iteraciones para converger
        for _ in range(5):
            T = now + dt
            pred = self.predict_eqc_position(T)
            dt = euclidean(pos_vqc, pred) / v_vqc


        angle   = math.radians(150)  # apertura de 30°
        spacing = 1.0               # 1 m entre cada “paso” de la V

        # Determinar ala y profundidad según el id (1…N)
        # lado: alterna izquierda/derecha; profundidad: ceil(id/2)

        # >>> BEGIN PATCH: lado/profundidad en base a rank local dentro del grupo
        rank = getattr(self, "formation_rank", 0)
        side  = -1 if (rank % 2) == 0 else 1    # alterna: izq/dcha
        depth = (rank // 2) + 1                  # 1,1,2,2,3,3,...
        # >>> END PATCH

        # Aproximar rumbo (heading) del EQC en este instante
        curr  = self.predict_eqc_position(now)
        fut   = self.predict_eqc_position(now + 0.1)
        heading = math.atan2(fut[1] - curr[1], fut[0] - curr[0])

        # Vector de offset en V
        dx = spacing * depth * math.cos(heading + side * angle)
        dy = spacing * depth * math.sin(heading + side * angle)

        # Punto final: intercept + offset en XY, misma Z
        return (pred[0] + dx, pred[1] + dy, config.h_vqc)

    # --- 3) Mantenimiento de modo satélite con intercepción dinámica ---
    def maintain_satellite_mode(self):
        """
        En lugar de un offset fijo, calcula el punto donde interceptar al EQC
        en movimiento y lanza la misión hacia allí.
        """
        # 1) calcular punto de interceptación
        intercept = self.compute_intercept()
        self.log.debug(f"🛰️ Satélite predictivo → interceptar en {intercept}")

        # 2) lanzar misión hacia ese punto SIN el argumento 'loop'
        #    (usa la configuración que ya diste en MissionMobilityConfiguration)
        self.mission.start_mission([intercept])

        # 3) cambiar estado
        self.state = "satellite"
        self._viz_push()

    #### [XY3D:helper]
    def _dist_and_warn_xy_vs_3d(self, coord3d: Tuple[float, float, float], urg: int) -> float:
        """
        Calcula distancias XY y 3D. Devuelve la distancia **XY** para evaluar llegada con R_DETECT.
        Si XY <= R_DETECT pero 3D > R_DETECT (por diferencia de altura), registra una alerta
        y guarda una muestra para diagnóstico.
        """
        dx = self.pos[0] - coord3d[0]
        dy = self.pos[1] - coord3d[1]
        dz = self.pos[2] - coord3d[2]
        dist_xy = math.hypot(dx, dy)
        dist_3d = math.sqrt(dx*dx + dy*dy + dz*dz)

        self.log.debug(f"    DistXY={dist_xy:.2f} Dist3D={dist_3d:.2f} (R_DETECT={config.R_DETECT})")

        # Aviso si la diferencia de altura bloquearía una llegada basada en 3D.
        if getattr(config, "WARN_XY_VS_3D", True) and (dist_xy <= config.R_DETECT < dist_3d):
            # Contadores/colección para diagnóstico
            self.xy3d_warns = getattr(self, "xy3d_warns", 0) + 1
            self.xy3d_samples = getattr(self, "xy3d_samples", [])

            # Buscar label de forma tolerante a float
            label_guess = None
            try:
                # tolerancia pequeña para coord XY
                tol = 1e-6
                cx, cy = coord3d[0], coord3d[1]
                label_guess = next(
                    p["label"] for p in config.POIS
                    if abs(p["coord"][0] - cx) < tol and abs(p["coord"][1] - cy) < tol and p["urgency"] == urg
                )
            except StopIteration:
                pass

            max_samp = int(getattr(config, "XY3D_MAX_SAMPLES", 5))
            if len(self.xy3d_samples) < max_samp:
                self.xy3d_samples.append({
                    "label": label_guess,
                    "coord": (coord3d[0], coord3d[1]),
                    "dist_xy": dist_xy,
                    "dist_3d": dist_3d,
                    "z_abs": abs(dz),
                    "R": config.R_DETECT,
                })

            self.log.debug(f"⚠️ XY ≤ R_DETECT pero 3D > R_DETECT: label={label_guess} coord={coord3d[:2]} "f"DistXY={dist_xy:.2f} Dist3D={dist_3d:.2f} | |dz|={abs(dz):.2f}. ""La llegada debe evaluarse en XY (objetivo en suelo).")

        return dist_3d #dist_xy  # clave: usamos XY para llegada

    #### [/XY3D:helper]

    def handle_telemetry(self, telemetry: Telemetry) -> None:
        self._exec["handle_telemetry"] = True
        in_mission = not self.mission.is_idle

        old = self.pos
        self.pos = telemetry.current_position
        self.log.debug(f"📡 Telemetry: from {old} to {self.pos}")

        for coord3d, urg in list(self.next2visit):
            dx, dy, dz = (
                self.pos[0] - coord3d[0],
                self.pos[1] - coord3d[1],
                self.pos[2] - coord3d[2]
            )
            dist = self._dist_and_warn_xy_vs_3d(coord3d, urg)
            self.log.debug(f"    Dist to {coord3d}: {dist:.2f} (tol={config.R_DETECT})")

            if dist <= config.R_DETECT:
                # encontramos el POI correspondiente:
                poi_id    = self.coordurg2id.get(((coord3d[0], coord3d[1]), urg))
                poi_label = self.coordurg2label.get(((coord3d[0], coord3d[1]), urg))

                # NEW: si ya fue colectado globalmente por otro VQC, salta y limpia misión
                if getattr(config, "USE_GLOBAL_COLLECTION_LOCK", False) and (poi_label in config.COLLECTED_LABELS):
                    entry = (coord3d, urg)
                    if entry in self.next2visit:
                        self.next2visit.remove(entry)
                        self.free = config.M - len(self.next2visit)
                        self._viz_push()

                        self.log.info(f"⛔ {poi_label} ya estaba colectado globalmente → lo quito de la misión")
                    continue  # no lo agregues al buffer

                # (tu código de siempre)
                already_discovered = any(d["id"] == poi_id for d in self.discovered)
                if poi_id not in self.visited and not already_discovered:
                    if len(self.discovered) < config.M:
                        #### [LATENCY] marca llegada real (assigned)
                        now = self.provider.current_time()
                        if poi_label not in self.arrival_ts:
                            self.arrival_ts[poi_label] = now
                            self.log.debug(f"⏱️ t_arrive[{poi_label}] = {now:.3f}s")
                        #### [/LATENCY]                        
                        # NEW: marcar candado global ANTES de añadir al buffer
                        if getattr(config, "USE_GLOBAL_COLLECTION_LOCK", False):
                            config.COLLECTED_LABELS.add(poi_label)
                            self.log.info(f"🔒 Lock global activado por VQC-{self.id} para {poi_label}")

                        # ➞ lo añadimos al buffer discovered (sin cambios)
                        self.discovered.append({"id": poi_id, "label": poi_label})

                        # ➞ clasificar y limpiar next2visit como ya hacías
                        entry = (coord3d, urg)
                        if entry in self.next2visit:
                            self.disc_assigned += 1
                            self.next2visit.remove(entry)
                            self.free = config.M - len(self.next2visit)

                            self._viz_push()
                            kind = "assigned"
                        else:
                            self.disc_casual += 1
                            kind = "casual"

                        self.log.info(f"🔍 Local detect ({kind}): {poi_id} ({poi_label})")
                    else:
                        self.log.debug("Buffer discovered lleno")

                # → Tras detectar uno assigned, puedes 'break' si solo esperas un PoI a la vez
                # break
        # 2) detección casual cuando no estamos en misión:
        if not self.next2visit:
            for poi in config.POIS:
                px, py = poi["coord"]
                dx, dy, dz = self.pos[0]-px, self.pos[1]-py, self.pos[2]-0.0

                dist = math.sqrt(dx*dx + dy*dy + dz*dz)

                if dist <= config.R_DETECT:
                    poi_id    = poi["id"]
                    poi_label = poi["label"]

                    # NEW: si ya está colectado globalmente, ignora
                    if getattr(config, "USE_GLOBAL_COLLECTION_LOCK", False) and (poi_label in config.COLLECTED_LABELS):
                        self.log.debug(f"⛔ Ya colectado globalmente: {poi_label} → ignoro (casual)")
                        continue

                    already_discovered = any(d["id"] == poi_id for d in self.discovered)
                    if poi_id not in self.visited and not already_discovered:
                        if len(self.discovered) < config.M:
                            #### [LATENCY] marca llegada real (casual)
                            now = self.provider.current_time()
                            if poi_label not in self.arrival_ts:
                                self.arrival_ts[poi_label] = now
                                self.log.debug(f"⏱️ t_arrive[{poi_label}] = {now:.3f}s (casual)")
                            #### [/LATENCY]
                            # NEW: marcar candado global ANTES de añadir al buffer
                            if getattr(config, "USE_GLOBAL_COLLECTION_LOCK", False):
                                config.COLLECTED_LABELS.add(poi_label)
                                self.log.info(f"🔒 Lock global activado por VQC-{self.id} para {poi_label}")

                            self.discovered.append({"id": poi_id, "label": poi_label})
                            self.disc_casual += 1
                            self.log.info(f"🔍 Casual detect: {poi_id} ({poi_label})")
                        else:
                            self.log.debug("Buffer discovered lleno")


    def handle_timer(self, timer: str) -> None:

        if timer == "hello":
            self._exec["handle_timer.hello"] = True
            """
            if self.discovered:
                report = {
                    "type": "DELIVER",
                    "v_id": self.id,
                    "pids": self.discovered.copy()
                }
                cmd = CommunicationCommand(
                    CommunicationCommandType.BROADCAST,
                    json.dumps(report)
                )
                self.provider.send_communication_command(cmd)
                self.log.info(f"📣 DELIVER automático en HELLO: entregados {self.discovered}")
                # Marcar como visitados y vaciar buffer
                #self.visited.extend(self.discovered)
                #self.discovered.clear()    """ 

                       
            free = config.M - len(self.next2visit)
            msg = {"type":"HELLO","v_id":self.id,"huecos":free,"position":list(self.pos)}

            self.free = free
            self._viz_push()

            self.log.debug(f"📤 HELLO payload: {msg}")
            cmd = CommunicationCommand(CommunicationCommandType.SEND, json.dumps(msg),self.leader_id)
            self.provider.send_communication_command(cmd)
            self.log.debug(f"📤 HELLO sent: free={free}")

            # === [LOG] Resumen del VQC en cada HELLO ===
            try:
                self.log.info(
                    f"[VQC-{self.id}] tick: state={self.state}, "
                    f"visited={len(self.visited)}, discovered={len(self.discovered)}, "
                    f"next2visit={len(self.next2visit)}, buffer={len(self.discovered)}"
                )
            except Exception as e:
                self.log.warning(f"[VQC-{self.id}] No se pudo loguear el resumen HELLO: {e}")
            # ===========================================


            self.provider.schedule_timer("hello", self.provider.current_time()+1)

        elif timer == "check_roam": #¿Estoy libre de misiones (mission.is_idle) y no estoy ya vagando de forma aleatoria (random._trip_ongoing)
            self.log.debug(f"🔥 check_roam: idle={self.mission.is_idle}")
            if self.mission.is_idle:
                if self.state == "visiting":
                    self.log.info("🏁 Fin de misión → modo satélite")
                    # === [LOG] Resumen al cerrar misión ===
                    try:
                        self.log.info(
                            f"[VQC-{self.id}] fin_mision: visited_total={len(self.visited)}, "
                            f"discovered_total={len(self.discovered)}, next2visit={len(self.next2visit)}"
                        )
                    except Exception as e:
                        self.log.warning(f"[VQC-{self.id}] No se pudo loguear fin_mision: {e}")
                    # ======================================
                    self.state = "satellite"
                self.maintain_satellite_mode()
                # si estoy en satellite y la misión idle, no hago nada
            self.provider.schedule_timer("check_roam", self.provider.current_time()+0.5)

    def handle_packet(self, message: str) -> None:
        self.log.debug(f"📥 handle_packet ASSIGN: {message}")
        msg = json.loads(message)

        t = msg.get("type")
        
        if t == "ASSIGN":
            self._exec["handle_packet.ASSIGN"] = True
            self.log.info(f"📥 ASSIGN received: {msg['pois']}")
 
#
#
#            # 1) Flush inmediato de PoIs descubiertos, garantizado por SEND
#            if self.discovered:
#                report = {
#                    "type": "DELIVER",
#                    "v_id": self.id,
#                    "pids": self.discovered.copy()
#               }
#                # Envío DIRECTO al EQC (asume id==0)
#                cmd_flush = CommunicationCommand(
#                    CommunicationCommandType.SEND,
#                   json.dumps(report),
#                    0
#                )
#                self.provider.send_communication_command(cmd_flush)
#                self.log.info(f"📣 DELIVER inmediato en ASSIGN: {self.discovered}")
                # NOTA: No borramos aquí; aguardamos al ACK

            antiguos = list(self.next2visit)

            self.next2visit.clear()
            # 3) Cargar primero las nuevas tareas que envía el EQC
            nuevos_ids = set()
            for p in msg["pois"]:
                x, y = p["coord"]
                urg = p["urgency"]
                coord3d = (x, y, 0.0) #config.h_vqc
                self.next2visit.append((coord3d, urg))
                nuevos_ids.add(p["label"])      # usa "label" o "id" según tu POIS
            # 4) Volver a añadir las antiguas que no estén ya en los nuevos,
            #    hasta completar la capacidad M

            for coord3d, urg in antiguos:
                # clave por (coord, urgency) para evitar ambigüedad
                key   = ((coord3d[0], coord3d[1]), urg)
                label = self.coordurg2label.get(key)
                if label not in nuevos_ids and len(self.next2visit) < config.M:
                    self.next2visit.append((coord3d, urg))

            # 5) Si tras el merge no queda nada, reanudar roaming
            if not self.next2visit:
                self.log.debug("🔄 ASSIGN vacío → sigo en modo satélite")
                self.free = config.M - len(self.next2visit)   # = M
                self._viz_push()                
                return
                
            # 6) Arrancar la misión guiada con la lista combinada
            coords  = [coord for (coord, _) in self.next2visit]
            self.log.debug(f"🗺️ Waypoints combinados: {coords}")
            self.state = "visiting"
            self.mission.start_mission(coords)
                        # Marca momento del último ASSIGN (opcional)
            try:
                self.last_assign_time = self.provider.current_time()
            except Exception:
                pass

            self.free = config.M - len(self.next2visit)
            # Actualiza variables para el visor
            self._viz_push()


            return
            
        elif t == "HELLO_ACK":
                self._exec["handle_packet.HELLO_ACK"] = True
                self.last_assign = {
                    "eqc_pos":  tuple(msg.get("eqc_pos", self.pos)),
                    "eqc_time": msg.get("eqc_time", self.provider.current_time())
                }
                self.log.debug(f"✅ VQC-{self.id} recebeu HELLO_ACK, enviando DELIVER en {self.last_assign['eqc_pos']} t={self.last_assign['eqc_time']}")
                self.send_deliver() 

        elif t == "DELIVER_ACK":
            self._exec["handle_packet.DELIVER_ACK"] = True
            acked = msg.get("pids", [])  # lista de IDs como strings
            self.log.info(f"📥 DELIVER_ACK recibido: {acked}")

            # Reemplaza tu loop antiguo por:
            for poi_id in acked:
                # quita cualquier dict con .["id"] == poi_id
                self.discovered = [d for d in self.discovered if d["id"] != poi_id]
                self.visited.append(poi_id)
                #### [LATENCY] cleanup de arrival_ts para el label correspondiente
                try:
                    label = next(p["label"] for p in config.POIS if p["id"] == poi_id)
                    self.arrival_ts.pop(label, None)
                except StopIteration:
                    pass
                #### [/LATENCY]
            self.log.debug(f"🗂️ discovered tras ACK: {self.discovered}, visited: {self.visited}")
            self._viz_push()
            # === [LOG] Resumen tras DELIVER_ACK ===
            try:
                pendientes_locales = max(0, len(self.discovered))
                self.log.info(
                    f"[VQC-{self.id}] deliver_ack: visited_total={len(self.visited)}, "
                    f"next2visit={len(self.next2visit)}, pendientes_locales={pendientes_locales}"
                )
            except Exception as e:
                self.log.warning(f"[VQC-{self.id}] No se pudo loguear post-ACK: {e}")
            # ======================================


        else:
            self.log.debug(f"⚠️ VQC-{self.id} recebeu mensagem desconhecida: {t}")

    def finish(self) -> None:
        self.log.info(f"🏁 VQC-{self.id} finished — next2visit={self.next2visit}, visited={self.visited}")
        self.log.info(f"📊 Discoveries: casual={self.disc_casual}, assigned={self.disc_assigned}")
        never = [k for k,v in self._exec.items() if not v]
        if never:
            self.log.warning(f"⚠️ Métodos VQC nunca ejecutados: {never}")
                #### [XY3D:finish]
        if getattr(config, "WARN_XY_VS_3D", True) and self.xy3d_warns:
            sample_str = ", ".join(
                f"(lab={s.get('label')}, xy={s['dist_xy']:.2f}, d3={s['dist_3d']:.2f}, |dz|={s['z_abs']:.2f})"
                for s in self.xy3d_samples
            )
            self.log.warning(
                f"⚠️ Discrepancias XY≤R pero 3D>R observadas {self.xy3d_warns} veces. "
                f"Ejemplos: {sample_str}"
            )
        #### [/XY3D:finish]

        

    def send_deliver(self) -> None:
        #### [DELIVER] no envíes si no hay nada que reportar
        if not self.discovered:
            self.log.debug("📭 DELIVER omitido: buffer 'discovered' vacío")
            return
        #### [/DELIVER]

        # Construye pids con t_arrive piggybacked
        now = self.provider.current_time()
        pids = []
        for d in self.discovered:
            label = d["label"]
            poi_id = d["id"]
            t_arr = self.arrival_ts.get(label)
            if t_arr is None:
                # Fallback defensivo (no debería ocurrir si marcamos al llegar)
                t_arr = now
            pids.append({"id": poi_id, "label": label, "t_arrive": t_arr})

        msg = {
            "type": "DELIVER",
            "v_id": self.id,
            "pids": pids
        }

        cmd = CommunicationCommand(
            CommunicationCommandType.SEND,
            json.dumps(msg),
            self.leader_id
        )
        self.provider.send_communication_command(cmd)
        self.log.info(f"📤 DELIVER enviado con t_arrive: {[e['id'] for e in pids]}")

        try:
            self.last_deliver_time = now
        except Exception:
            pass
        self._viz_push()
