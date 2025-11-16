"""
Exploration Quadcopter (E-QC) protocol:
- Patrols the area via predefined waypoints.
- Captures and filters PoI detections.
- Coordinates with V-QCs by sending ASSIGN messages.
- Limits total ASSIGNs per physical encounter (not per timer tick)
"""

import json                                                   
import math                                                  
import logging
from typing import List
from collections import Counter        #

from gradysim.protocol.interface import IProtocol
from gradysim.protocol.messages.communication import CommunicationCommand, CommunicationCommandType
from gradysim.protocol.messages.telemetry import Telemetry
from gradysim.protocol.plugin.mission_mobility import MissionMobilityPlugin, MissionMobilityConfiguration, LoopMission
from gradysim.simulator.extension.camera import CameraHardware, CameraConfiguration

import config
from config import MAX_ASSIGN_PER_ENCOUNTER
from config import EQC_WAYPOINTS 
# --- dentro de EQCProtocol ---

class EQCProtocol(IProtocol):

    def initialize(self) -> None:
        self.id = self.provider.get_id()
        self.log = logging.getLogger(f"EQC-{self.id}")
        self.log.info(f"Current handlers: s{self.log.handlers}")
        self.assignment_policy = config.ASSIGNMENT_POLICY # or "round_robin" or "load_balancing"  greedy
        self.encounter_assigned = {vid: 0 for vid in range(config.NUM_VQCS)}
        self.last_hello_time = {}
        # === Assignment scheduler (Pattern B) ===
        self._assign_triggered = False          # flag coalescido de "quiero asignar"
        self._next_assign_earliest = 0.0        # cooldown opcional (timestamp sim en segundos)

        self._last_wp = None
        self.pos = (0.0, 0.0, 0.0)

        #### [LATENCY] nuevas métricas desglosadas
        self.lat_service = []    # (label, t_arrive - t_detect)
        self.lat_contact = []    # (label, t_deliver_ack - t_arrive)
        self.lat_e2e     = []    # (label, t_deliver_ack - t_spawn)  # t_spawn ~ self.start_time
        self.t_detect_list = []  # (t_detect - t_spawn)
        #### [/LATENCY]

        waypoints = config.EQC_WAYPOINTS[self.id]
        if self.id == 0:
            self.log.info(f"[WAYPOINTS] EQC-{self.id} → {len(waypoints)} puntos")
            for i, wp in enumerate(waypoints):
                self.log.info(f"[WAYPOINTS] EQC-{self.id}[{i}] = {tuple(wp)}")

        self.log.info(f"🛰️  EQC iniciando patrulla con waypoints: {waypoints}")
        cfg = MissionMobilityConfiguration(
            speed=config.EQC_SPEED,
            loop_mission=LoopMission.RESTART,
            tolerance=1
        )
        self.mission = MissionMobilityPlugin(self, cfg)
        self.mission.start_mission(waypoints)
        self.log.info(f"🛰️ EQC-{self.id} patrulla {len(waypoints)} puntos")

        # ---------- Métricas ----------
        self.start_time        = self.provider.current_time()
        self.assign_count      = 0                # total ASSIGNs enviadas
        self.assign_success    = 0
        self.global_score      = 0                # PoIs de ASSIGN que efectivamente se entregaron
        self.assign_times      = {}               # mapa poi_label → t_assign
        self.latencies         = []               # lista de (label, latency)
        self.coverage_timeline = []               # lista de (elapsed_time, unique_count)
        self.redundant_delivers = 0
        
        config.METRICS.setdefault("unique_ids", set())
        config.METRICS.setdefault("redundant", 0)
        config.METRICS.setdefault("global_score", 0)
        
        self.cam_raw_count     = 0   # cada nodo detectado por take_picture()
        self.cam_poi_matches   = 0   # cuántos de esos nodes eran PoIs
        # Flags
        self._executed = {
            "handle_timer.assign": False,
            "handle_packet.HELLO": False,
            "handle_packet.DELIVER": False,
        }
    
        # Configurar cámara
        cam_cfg = CameraConfiguration(
            camera_reach=config.R_CAMERA,
            camera_theta=180.0, #########################no filtra por anguñp
            facing_elevation=180.0,
            facing_rotation=0.0
        )
        self.camera = CameraHardware(self, cam_cfg)
        self.log.info(f"📷 Camera configured: reach={config.R_CAMERA}, theta={cam_cfg.camera_theta}")

        # Estados internos
        self.pending: List[dict] = []
        self.detect_ts: dict = {}
        self.vqc_states: dict = {}
        self.assign_counts = {}  
        # Programar muestreo de detección y asignación cada 1s
        next_t = self.provider.current_time() + 1
        self.provider.schedule_timer("assign", next_t)
                # === [LOG] Resumen inicial del EQC ===
        try:
            self.log.info(
                f"[EQC-{self.id}] init: pending={len(self.pending)}, "
                f"assigns={self.assign_count}, vqcs={len(self.vqc_states)}"
            )
        except Exception as e:
            self.log.warning(f"[EQC-{self.id}] init log failed: {e}")
        # =====================================

        self.log.info(f"✅ First ‘assign’ timer scheduled for t={next_t:.2f}s")
        self.log.debug(f"⏱️ Scheduled first 'assign' at t={next_t:.2f}")

    # === [VISUALIZACIÓN] Publicar variables al panel ===""""
    def _viz_push(self) -> None:
        try:
            tv = self.provider.tracked_variables

            # Identidad del líder (EQC)
            try:
                tv["eqc_id"] = int(self.provider.get_id())
            except Exception:
                tv["eqc_id"] = None

            # Métricas globales (conserva lo que ya publicabas)
            uniq = config.METRICS.get("unique_ids", set())
            tv["ids_collected"] = sorted(list(uniq))                 # lista de labels únicos
            tv["unique_count"]  = len(uniq)                          # cuántos únicos
            tv["redundant"]     = int(config.METRICS.get("redundant", 0))

            # Contadores de asignación en este EQC
            tv["assigns"] = int(getattr(self, "assign_count", 0))

            # Cola local de detecciones pendientes de asignar
            try:
                pending_list = [
                    (p.get("label") if isinstance(p, dict) and "label" in p else str(p))
                    for p in getattr(self, "pending", [])
                ]
            except Exception:
                pending_list = []
            tv["pending"]        = pending_list[:200]  # recorte defensivo
            tv["pending_count"]  = len(pending_list)   # útil para ver picos sin abrir la lista
            tv["eqc_pending"]    = tv["pending_count"] # alias cómodo

            # Último 'free' reportado por cada VQC (HELLO)
            vqc_free = {}
            for vid, st in getattr(self, "vqc_states", {}).items():
                # normaliza ID a int si es posible
                try:
                    vid_i = int(vid)
                except Exception:
                    try:
                        vid_i = int(st.get("id", -1))
                    except Exception:
                        vid_i = None
                if vid_i is None:
                    continue

                # algunos setups usan 'huecos', otros 'free' o 'slots'
                free_val = st.get("free", st.get("huecos", st.get("slots", 0)))
                try:
                    vqc_free[vid_i] = int(free_val)
                except Exception:
                    vqc_free[vid_i] = 0

            tv["vqc_free"]      = vqc_free              # mantiene tu clave original
            tv["eqc_vqc_free"]  = dict(vqc_free)        # alias con prefijo eqc_ para el panel

            # Distribución de asignaciones por VQC (balanceo)
            assign_counts = getattr(self, "assign_counts", {})
            if isinstance(assign_counts, dict):
                tv["eqc_assign_counts"] = {int(k): int(v) for k, v in assign_counts.items()}
            else:
                tv["eqc_assign_counts"] = {}

        except Exception as e:
            self.log.debug(f"[viz][EQC] publish error: {e}")

    # === Assignment scheduler (Pattern B) ===
    def trigger_assign(self, reason: str) -> None:
        """Marca intención de asignar; no asigna aquí."""
        self._assign_triggered = True
        self.log.debug(f"trigger_assign ← {reason} (pending={len(getattr(self,'pending',[]))}, vqc_states={len(getattr(self,'vqc_states',{}))})")


    def _any_vqc_has_free(self) -> bool:
        """¿Algún VQC reporta huecos > 0 en su HELLO? Usa la última info en vqc_states."""
        for st in self.vqc_states.values():
            if st.get("huecos", 0) > 0:
                return True
        return False

    def _should_assign_now(self, now: float) -> bool:
        
        """¿Conviene ejecutar assign_to_vqcs() en este tick del timer?"""

        if not self._assign_triggered:
            self.log.debug("⏸️ no-assign: _assign_triggered=False")
            return False
        if not self.pending:
            self.log.debug("⏸️ no-assign: pending vacío")
            return False
        if not self.vqc_states:
            self.log.debug("⏸️ no-assign: no hay VQCs con estado (aún no HELLO)")
            return False
        if now < getattr(self, "_next_assign_earliest", 0.0):
            self.log.debug(f"⏸️ no-assign: cooldown hasta t={self._next_assign_earliest:.2f}")
            return False        
        # Al menos un VQC con huecos reportados
        return self._any_vqc_has_free()

    def handle_telemetry(self, telemetry: Telemetry) -> None: # lo que hace es imprimir posicion y a que waypoint se dirige
        self.log.debug(f"📡 Telemetry: pos={telemetry.current_position}, idle={self.mission.is_idle}")
        self.pos = telemetry.current_position
        if not self.mission.is_idle:
            wp = self.mission.current_waypoint
            self.log.debug(f"🛰️ EQC moving towards waypoint {wp}")
            # INFO sólo cuando cambiamos de waypoint
            if wp != self._last_wp:
                self.log.info(f"🛰️ EQC rumbo al waypoint en {wp}")
                self._last_wp = wp

    def handle_timer(self, timer: str) -> None: # lo que hace es actualizar self.pending con las coordenadas detectadas
        self.log.debug(f"handle_timer invoked with timer='{timer}'")
        if timer == "assign":
            self._executed["handle_timer.assign"] = True
            now = self.provider.current_time()
            self.log.info("*"*40 + f" t={now:.2f}s " + "*"*40)
            ########
            self.log.info(f"⚙️  EQC handle_timer('assign') @ t={now:.2f}")

            # [ADD] — Antes de todo: purga de pendientes ya bloqueados globalmente
            if getattr(config, "USE_GLOBAL_COLLECTION_LOCK", False):
                before = len(self.pending)
                self.pending = [p for p in self.pending if p["label"] not in config.COLLECTED_LABELS]
                removed = before - len(self.pending)
                if removed > 0:
                    self.log.debug(f"🧹 Pruned {removed} pending by global lock")
            # [STATE] foto de estado al entrar
            try:
                vqc_free_snapshot = {int(k): int(v.get("huecos",0)) for k, v in self.vqc_states.items()}
            except Exception:
                vqc_free_snapshot = {}
            self.log.debug(
                f"[EQC-{self.id}] assign_in: pending={len(self.pending)}, "
                f"vqc_free={vqc_free_snapshot}, triggered={self._assign_triggered}"
            )

            detected = self.camera.take_picture()
            # Métrica raw
            self.cam_raw_count += len(detected)
            self.log.debug(f"⚙️  assign @ t={now:.2f}: {len(detected)} nodos detectados")

            # Log raw detections (agrupados)
            self._log_raw_detections(detected)

            # Filtrar PoIs
            new_cnt = 0
            eps = 0.2
            for poi in config.POIS:
                px, py = poi["coord"]
                for node in detected:
                    x, y, z = node["position"]
                    if abs(x-px)<eps and abs(y-py)<eps and abs(z-0.0)<eps:
                        label = poi["label"]

                        # [ADD] — Si el VQC ya “bloqueó” globalmente este PoI, no lo metas a pending
                        if getattr(config, "USE_GLOBAL_COLLECTION_LOCK", False) and (label in config.COLLECTED_LABELS):
                            self.log.debug(f"⛔ Skip adding {label}: globally collected by a VQC")
                            break  # pasamos al siguiente poi

                        # Tu lógica original para registrar detección válida 1ª vez
                        if label not in self.detect_ts:
                            self.cam_poi_matches += 1
                            self.detect_ts[label] = now

                            self.t_detect_list.append(now - self.start_time)

                            # [ADD] — Evitar duplicados en pending por seguridad (por label)
                            if all(p["label"] != label for p in self.pending):
                                self.pending.append(poi)
                            new_cnt += 1
                            self.log.info(f"🔍 {label} detectado @ {poi['coord']} t={now:.2f}")
                        break  # ya casó con este poi, sale del for node

            self.log.debug(f"🗂️ pending size /relacionado con new_cnt: {len(self.pending)} (+{new_cnt})")
            self.log.debug(
                f"[EQC-{self.id}] detect_summary: new_pending={new_cnt}, "
                f"pending_now={len(self.pending)}"
            )
            
            # Reprogramar
            next_t = now + 1
            self.provider.schedule_timer("assign", next_t)
            self.log.debug(f"⏱️ Rescheduled 'assign' at t={next_t:.2f}")

            now = self.provider.current_time()
            if self._should_assign_now(now):

                self._next_assign_earliest = now + 0.1 #100ms
                self._assign_triggered = False
                self.log.info(
                    f"[EQC-{self.id}] assign_exec: pending_pre={len(self.pending)}, "
                    f"vqcs={len(self.vqc_states)}"
                )
                self.assign_to_vqcs()
                self.log.info(
                    f"[EQC-{self.id}] assign_done: pending_post={len(self.pending)}, "
                    f"assigns_total={self.assign_count}"
                )
            else:
                # Solo loguea el motivo si había intención real de asignar
                if self._assign_triggered:
                    reason = []
                    if not self.pending: reason.append("pending=0")
                    if not self.vqc_states: reason.append("sin_hello")
                    if now < getattr(self, "_next_assign_earliest", 0.0):
                        reason.append(f"cooldown_until={self._next_assign_earliest:.2f}")
                    if not reason and not self._any_vqc_has_free():
                        reason.append("sin_huecos_vqc")
                    #self.log.info(f"[EQC-{self.id}] assign_skip: {';'.join(reason) or 'condición_no_cumplida'}")
                    msg = f"[EQC-{self.id}] assign_skip: {';'.join(reason) or 'condición_no_cumplida'}"
                    if "sin_huecos_vqc" in reason:
                        try:
                            vqc_free_snapshot = {int(k): int(v.get("huecos",0)) for k, v in self.vqc_states.items()}
                        except Exception:
                            vqc_free_snapshot = {}
                        msg += f" | vqc_free={vqc_free_snapshot}"
                    self.log.info(msg)


    def handle_packet(self, message: str) -> None: #se activa con HELLO o deliver, actualiza vqc states, pendindg      y en deliver
        self.log.debug(f"📥 [RAW] handle_packet recibido: {message}")
        msg = json.loads(message)
        t = msg.get("type")
        vid = msg["v_id"]
        # Si aún no tenemos estado de este VQC y el mensaje no es HELLO, lo ignoramos
        if t != "HELLO" and vid not in self.vqc_states:
            self.log.warning(f"Ignorando {t} de VQC-{vid} (no hay HELLO previo)")
            return
        if t == "HELLO":
            self._executed["handle_packet.HELLO"] = True
            now = self.provider.current_time()
            prev = self.last_hello_time.get(vid)
            if prev is None or (now - prev) > 1.2:
                self.encounter_assigned[vid] = 0
            self.last_hello_time[vid] = now

            free = msg["huecos"]
            pos = tuple(msg["position"])
            self.log.debug(f"📩 HELLO from VQC-{vid}: free={free}, pos={pos}")
            #before = len(self.pending)
            #self.pending = [p for p in self.pending if p["label"] not in visited]
            #self.log.debug(f"🗑️ pending filtered: {before}→{len(self.pending)}")
            self.vqc_states[vid] = {"huecos": free, "pos": pos}
            self._viz_push()
            # Resumen por HELLO (si no asignamos es por falta de pending / cooldown / sin huecos)
            try:
                self.log.debug(
                    f"[EQC-{self.id}] hello: vqc={vid}, free={free}, "
                    f"pending={len(self.pending)}, triggered={self._assign_triggered}"
                )
            except Exception:
                pass

            if free <= 0:
                    self.log.debug(f"→ VQC-{vid} buffer FULL tras assign")

            ack = {"type": "HELLO_ACK", "v_id": vid, "eqc_id": self.id ,"eqc_pos": list(self.pos), "eqc_time": self.provider.current_time()}
            cmd_ack = CommunicationCommand(
                CommunicationCommandType.SEND,
                json.dumps(ack),
                vid
            )
            self.provider.send_communication_command(cmd_ack)
            self.log.info(f"📣 EQC envió HELLO_ACK a VQC-{vid}")

            if self.pending and free > 0:
                self.log.info(f"[EQC-{self.id}] hello_unlock: vqc={vid}, free={free}, pending={len(self.pending)}")

                self.trigger_assign("HELLO")

        elif t == "DELIVER":
            self._executed["handle_packet.DELIVER"] = True
            now = self.provider.current_time()
            vid = msg["v_id"]
            delivered = msg.get("pids", [])  
            self.log.info(f"📥 DELIVER from VQC-{vid}: {delivered}")
            for entry in delivered:
                label = entry.get("label")
                poi_id = entry.get("id")
                if label is None or poi_id is None:
                    self.log.warning(f"DELIVER malformed: {entry!r}")
                    continue

                #### [LATENCY:calc] — nuevas métricas desglosadas
                t_arrive = entry.get("t_arrive")   # viene piggybacked desde el VQC
                t_detect = self.detect_ts.get(label)
                if t_arrive is not None and t_detect is not None:
                    self.lat_service.append((label, t_arrive - t_detect))   # “servicio” real
                    self.lat_contact.append((label, now - t_arrive))        # overhead de contacto
                else:
                    self.log.debug(f"⏱️ métricas parciales: t_arrive={t_arrive}, t_detect={t_detect} para {label}")
                self.lat_e2e.append((label, now - self.start_time))
                #### [/LATENCY:calc]

                t0 = self.assign_times.pop(label, None)
                if t0 is not None:
                    latency = now - t0
                    self.latencies.append((label, latency))
                    self.assign_success += 1
                    poi = next(p for p in config.POIS if p["label"] == label or p["id"] == poi_id)
                    w = config.URGENCY_WEIGHTS.get(poi["urgency"], 0)
                    self.global_score += w
                    if label not in config.METRICS["unique_ids"]:
                        config.METRICS["unique_ids"].add(label)

                    #### [FRESHNESS:optional bump free slots]
                    if getattr(config, "BUMP_FREE_ON_ASSIGNED_DELIVER", False):
                        st = self.vqc_states.get(vid)
                        if st:
                            st["huecos"] = min(config.M, st.get("huecos", 0) + 1)
                            self.log.debug(f"↗️ huecos(VQC-{vid}) += 1 (now {st['huecos']}) por DELIVER asignado de {label}")
                    #### [/FRESHNESS:optional bump free slots]

                elif label not in config.METRICS["unique_ids"]:
                    config.METRICS["unique_ids"].add(label)
                    poi = next((p for p in config.POIS if p["label"] == label or p["id"] == poi_id), None)
                    if poi: 
                        self.global_score += config.URGENCY_WEIGHTS.get(poi["urgency"], 0)
                    self.log.debug(f"ℹ️ First auto‐deliver for {label}")
                else:
                    self.redundant_delivers += 1
                    config.METRICS["redundant"] += 1
                    self._viz_push()

                    self.log.debug(f"⚠️ Redundant DELIVER for {label}")

                elapsed = now - self.start_time
                self.coverage_timeline.append((elapsed, len(config.METRICS["unique_ids"])))

            self.log.debug(f"🧮 Metrics: unique={len(config.METRICS['unique_ids'])}, redundant={config.METRICS['redundant']}")
            delivered_labels = [e["label"] for e in delivered]
            self.log.debug(f"DELIVER recibido de VQC-{vid}: {delivered_labels}")
            self.pending = [p for p in self.pending if p["label"] not in delivered_labels]
            self._viz_push()

            # ACK al VQC
            ack_payload = {
                "type": "DELIVER_ACK",
                "v_id": vid,
                "pids": [entry["id"] for entry in delivered]
            }
            cmd_ack = CommunicationCommand(
                CommunicationCommandType.SEND,
                json.dumps(ack_payload),
                vid
            )
            self.provider.send_communication_command(cmd_ack)
            self.log.info(f"📣 Enviado DELIVER_ACK a VQC-{vid}: {ack_payload['pids']}")

            self.trigger_assign("DELIVER")

    def assign_to_vqcs(self) -> None:
        if self.assignment_policy == "greedy":
            self._assign_greedy()
        elif self.assignment_policy == "round_robin":
            self._assign_round_robin()
        elif self.assignment_policy == "load_balancing":
            self._assign_load_balancing()
        else:
            self.log.error(f"Unknown assignment policy: {self.assignment_policy}")

    ########### editar aqui ###########
    # Método para política Greedy (tu implementación actual)
    def _assign_greedy(self) -> None:
        now = self.provider.current_time()
        self.log.debug(f"🔍 assign_to_vqcs (Greedy): pending={len(self.pending)}, states={self.vqc_states}")

        if not self.vqc_states or not self.pending:
            self.log.debug("→ No VQCs or no PoIs pending")
            return

        for vid, st in self.vqc_states.items():
            free, pos = st["huecos"], st["pos"]
            self.log.debug(f"→ VQC-{vid} state: free={free}, pos={pos}")
            if free <= 0 or self.encounter_assigned.get(vid, 0) >= MAX_ASSIGN_PER_ENCOUNTER:
                self.log.debug(f"→ VQC-{vid} no free slots / {MAX_ASSIGN_PER_ENCOUNTER} reached")
                continue

            # [ADD] — Candidatos filtrados por candado global (re-evaluado por VQC)
            if getattr(config, "USE_GLOBAL_COLLECTION_LOCK", False):
                candidates = [p for p in self.pending if p["label"] not in config.COLLECTED_LABELS]
            else:
                candidates = list(self.pending)

            if not candidates:
                self.log.debug("→ No PoIs candidates after global-lock filter")
                continue

            scored = []
            for poi in candidates:
                dist = max(1e-6, math.hypot(pos[0] - poi["coord"][0], pos[1] - poi["coord"][1]))
                score = poi["urgency"] / dist
                scored.append((score, poi))
                self.log.debug(f"    ⋅ {poi['label']} urg={poi['urgency']} dist={dist:.2f} score={score:.2f}")

            scored.sort(key=lambda x: x[0], reverse=True)
            remaining = MAX_ASSIGN_PER_ENCOUNTER - self.encounter_assigned.get(vid, 0)
            limit = min(free, remaining)
            to_assign = [p for _, p in scored[:limit]]

            # [ADD] — Re-chequeo de carrera
            if getattr(config, "USE_GLOBAL_COLLECTION_LOCK", False):
                to_assign = [p for p in to_assign if p["label"] not in config.COLLECTED_LABELS]
            if not to_assign:
                self.log.debug(f"→ No PoIs for VQC-{vid} after race-check")
                continue

            for p in to_assign:
                self.assign_times[p["label"]] = now
                if p in self.pending:
                    self.pending.remove(p)
            self.assign_count += len(to_assign)

            payload = {
                "type": "ASSIGN", "v_id": vid,
                "pois": [{"label": p["label"], "coord": p["coord"], "urgency": p["urgency"], "ts": self.detect_ts[p["label"]]} for p in to_assign]
            }
            self.log.debug(f"🚀 ASSIGN payload for VQC-{vid} (Greedy): {payload}")

            cmd = CommunicationCommand(CommunicationCommandType.SEND, json.dumps(payload), vid)
            self.provider.send_communication_command(cmd)
            self.assign_counts[vid] = self.assign_counts.get(vid, 0) + len(to_assign)

            self.encounter_assigned[vid] = self.encounter_assigned.get(vid, 0) + len(to_assign)

            self.log.info(f"🚀 ASSIGN {len(to_assign)} to VQC-{vid}: {[p['label'] for p in to_assign]}")
            self.vqc_states[vid]["huecos"] -= len(to_assign)


    ########### editar aqui ###########
    # Método para política Round-Robin
    def _assign_round_robin(self) -> None:
        now = self.provider.current_time()
        self.log.debug(f"🔍 assign_to_vqcs (Round-Robin): pending={len(self.pending)}, states={self.vqc_states}")

        if not hasattr(self, "_rr_index"):
            self._rr_index = 0

        vqc_ids = list(self.vqc_states.keys())
        n_vqcs = len(vqc_ids)
        if n_vqcs == 0:
            self.log.debug("→ No VQCs available for assignment")
            return

        # [ADD] — Candidatos filtrados por candado global
        if getattr(config, "USE_GLOBAL_COLLECTION_LOCK", False):
            candidates = [p for p in self.pending if p["label"] not in config.COLLECTED_LABELS]
        else:
            candidates = list(self.pending)

        if not candidates:
            self.log.debug("→ No PoIs candidates after global-lock filter")
            return

        for _ in range(n_vqcs):
            vid = vqc_ids[self._rr_index % n_vqcs]
            self._rr_index += 1

            st = self.vqc_states[vid]
            free, pos = st["huecos"], st["pos"]

            self.log.debug(f"→ VQC-{vid} state: free={free}, pos={pos}")
            if free <= 0:
                self.log.debug(f"→ VQC-{vid} no free slots")
                continue

            if not candidates:
                self.log.debug("→ No PoIs pending/candidates")
                break

            to_assign = [candidates[0]]  # Solo un PoI por ronda

            # [ADD] — Re-chequeo de carrera
            if getattr(config, "USE_GLOBAL_COLLECTION_LOCK", False):
                to_assign = [p for p in to_assign if p["label"] not in config.COLLECTED_LABELS]
            if not to_assign:
                self.log.debug("→ Candidate got collected meanwhile; continue RR")
                continue

            for p in to_assign:
                self.assign_times[p["label"]] = now
                if p in self.pending:
                    self.pending.remove(p)
                if p in candidates:
                    candidates.remove(p)
            self.assign_count += len(to_assign)

            payload = {
                "type": "ASSIGN", "v_id": vid,
                "pois": [{"label": p["label"], "coord": p["coord"], "urgency": p["urgency"], "ts": self.detect_ts[p["label"]]} for p in to_assign]
            }
            self.log.debug(f"🚀 ASSIGN payload for VQC-{vid} (Round-Robin): {payload}")

            cmd = CommunicationCommand(CommunicationCommandType.SEND, json.dumps(payload), vid)
            self.provider.send_communication_command(cmd)
            self.assign_counts[vid] = self.assign_counts.get(vid, 0) + len(to_assign)

            self.log.info(f"🚀 ASSIGN {len(to_assign)} to VQC-{vid}: {[p['label'] for p in to_assign]}")

            self.vqc_states[vid]["huecos"] -= len(to_assign)
            break  # Asigna solo 1 VQC por llamada (igual que tu versión)

    ########### editar aqui ###########
    # Método para política Load-Balancing
    def _assign_load_balancing(self) -> None:

  

        now = self.provider.current_time()
        self.log.debug(f"🔍 assign_to_vqcs (Load-Balancing): pending={len(self.pending)}, states={self.vqc_states}")

        if not self.vqc_states or not self.pending:
            self.log.debug("→ No VQCs or no PoIs pending")
            return

        # Candidatos (filtro inicial por lock global si aplica)
        if getattr(config, "USE_GLOBAL_COLLECTION_LOCK", False):
            candidates = [p for p in self.pending if p["label"] not in config.COLLECTED_LABELS]
        else:
            candidates = list(self.pending)

        if not candidates:
            self.log.debug("→ No PoIs candidates after global-lock filter")
            return

        # Asignación por rondas: cada ronda intenta dar ≤1 PoI a cada VQC elegible (ordenados por free desc).
        while candidates:
            # Elegibles = VQCs con huecos y sin superar el throttle del encuentro
            eligibles = []
            for vid, st in self.vqc_states.items():
                free = int(st.get("huecos", 0))
                remaining = MAX_ASSIGN_PER_ENCOUNTER - self.encounter_assigned.get(vid, 0)
                if free > 0 and remaining > 0:
                    eligibles.append((vid, free))

            if not eligibles:
                self.log.debug("→ No VQC con slots o throttle disponible")
                self.log.info(
                    f"[EQC-{self.id}] lb:no_assign (pending={len(self.pending)}, "
                    f"candidates={len(candidates)}, eligibles=0)"
                )

                break


            # Ordena por mayor free (balanceo real)
            eligibles.sort(key=lambda x: x[1], reverse=True)

            any_assigned_this_round = False

            for vid, _free in eligibles:
                st = self.vqc_states[vid]
                free = int(st.get("huecos", 0))
                remaining = MAX_ASSIGN_PER_ENCOUNTER - self.encounter_assigned.get(vid, 0)
                if free <= 0 or remaining <= 0:
                    continue

                pos = st["pos"]

                # Elige el mejor PoI para ESTE VQC (urgency/dist)
                best = None
                best_score = -1.0
                for poi in candidates:
                    # Re-chequeo carrera/lock por si otro VQC lo marcó
                    if getattr(config, "USE_GLOBAL_COLLECTION_LOCK", False) and (poi["label"] in config.COLLECTED_LABELS):
                        continue
                    dist = max(1e-6, math.hypot(pos[0] - poi["coord"][0], pos[1] - poi["coord"][1]))
                    score = poi["urgency"] / dist
                    if score > best_score:
                        best_score = score
                        best = poi

                if best is None:
                    continue  # no hay candidato utilizable para este VQC

                # Asigna SOLO 1 a este VQC en esta ronda
                self.assign_times[best["label"]] = now
                if best in self.pending:
                    self.pending.remove(best)
                if best in candidates:
                    candidates.remove(best)
                self.assign_count += 1
                self.assign_counts[vid] = self.assign_counts.get(vid, 0) + 1
                self.encounter_assigned[vid] = self.encounter_assigned.get(vid, 0) + 1
                st["huecos"] = max(0, free - 1)

                payload = {
                    "type": "ASSIGN",
                    "v_id": vid,
                    "pois": [{
                        "label": best["label"],
                        "coord": best["coord"],      # ⇐ usa coord (x,y) del PoI; el VQC añade su z
                        "urgency": best["urgency"],
                        "ts": self.detect_ts[best["label"]],
                    }],
                }
                self.log.debug(f"🚀 ASSIGN payload for VQC-{vid} (LB): {payload}")
                cmd = CommunicationCommand(CommunicationCommandType.SEND, json.dumps(payload), vid)
                self.provider.send_communication_command(cmd)
                self.log.info(f"🚀 ASSIGN 1 to VQC-{vid}: {best['label']}")

                any_assigned_this_round = True
                if not candidates:
                    break  # ya no queda nada que repartir

            if not any_assigned_this_round:
                self.log.debug("→ Ninguna asignación posible en esta ronda; salgo")
                self.log.info(
                    f"[EQC-{self.id}] lb:no_assign (pending={len(self.pending)}, "
                    f"candidates_left={len(candidates)})"
                )                
                break

    def finish(self) -> None:
        # === Latencia principal del paper: L_service = t_arrive - t_detect
        Ls = [x for _, x in self.lat_service]
        avg_service = (sum(Ls) / len(Ls)) if Ls else float('nan')

        self.log.info(f"✔️ assign_success    = {self.assign_success}")
        self.log.info(f"ℹ️ redundant_delivers = {self.redundant_delivers}")
        self.log.info(f"⏱️ avg_service_latency = {avg_service:.3f}s  (t_arrive - t_detect)")

        # (opcional, solo como métrica de depuración)
        A2A = [x for _, x in self.latencies]
        if A2A:
            self.log.debug(f"(debug) avg_assign_to_ack = {sum(A2A)/len(A2A):.3f}s")
        # ... resto del finish ...


        total_time = self.provider.current_time() - self.start_time
        unique = len(config.METRICS["unique_ids"])
        redundant = config.METRICS["redundant"]
        success = self.assign_success
        assigns = self.assign_count
        avg_latency = sum(l for _, l in self.latencies) / len(self.latencies) if self.latencies else float('nan')
        discovery_rate = unique / total_time if total_time>0 else float('nan')
        success_rate   = success / assigns if assigns>0 else float('nan')

        self.log.info(f"✅ EQC finished. Unique={unique}, redundant={redundant}")
        self.log.info(f"   Assigns sent={assigns}, successful delivers={success} (rate={success_rate:.2f})")
        self.log.info(f"   Avg. service latency={avg_service:.2f}s, discovery rate={discovery_rate:.2f} PoIs/s")
        self.log.info(f"⭐ Global mission score = {self.global_score:.2f}")
        config.METRICS["global_score"] += self.global_score
        self.log.info(f"📷 Cámara hizo {self.cam_raw_count} detecciones totales, "
                      f"{self.cam_poi_matches} coincidencias con PoIs")
       
        never_called = [k for k,v in self._executed.items() if not v]
        if never_called:
            self.log.warning(f"⚠️ Métodos nunca ejecutados: {never_called}")
        #### [LATENCY] resumen
        def p95(vals):
            if not vals:
                return float('nan')
            vs = sorted(vals)
            return vs[int(0.95 * (len(vs) - 1))]

        mean = lambda v: (sum(v) / len(v)) if v else float('nan')

        Ls   = [x for _, x in self.lat_service]
        Lc   = [x for _, x in self.lat_contact]
        Le2e = [x for _, x in self.lat_e2e]

        self.log.info(f"⏱️ service_latency: mean={mean(Ls):.3f}s p95={p95(Ls):.3f}s  (t_arrive - t_detect)")
        self.log.info(f"📡 contact_overhead: mean={mean(Lc):.3f}s p95={p95(Lc):.3f}s  (t_deliver_ack - t_arrive)")
        self.log.info(f"🔚 e2e_latency:      mean={mean(Le2e):.3f}s p95={p95(Le2e):.3f}s  (t_deliver_ack - t_spawn)")
        self.log.info(f"👁️ time_to_detect:   mean={mean(self.t_detect_list):.3f}s p95={p95(self.t_detect_list):.3f}s  (t_detect - t_spawn)")
        # --- Acumular al global ---
        try:
            config.METRICS["lat_service_all"].extend([x for _, x in self.lat_service])
            config.METRICS["lat_contact_all"].extend([x for _, x in self.lat_contact])
            config.METRICS["lat_e2e_all"].extend([x for _, x in self.lat_e2e])
            config.METRICS["t_detect_all"].extend(self.t_detect_list)

            config.METRICS["cam_raw_all"]  += self.cam_raw_count
            config.METRICS["cam_hits_all"] += self.cam_poi_matches

            config.METRICS["eqc_reports"].append({
                "eqc_id": self.id,
                "assigns": self.assign_count,
                "success": self.assign_success,
            })
            emit_tables_and_glossary(self)

        except Exception as e:
            self.log.debug(f"[summary] accumulation error: {e}")

        self.log.info(f"[EQC-{self.id}] finish: pending_final={len(self.pending)}, vqcs={len(self.vqc_states)}")
            


    def _log_raw_detections(self, detected: List[dict]):
        """
        Agrupa y logea posiciones únicas de las detecciones en un solo mensaje.
        """
        if not detected:
            return
        
        coords = [tuple(n["position"]) for n in detected]
        counts = Counter(coords)
        entries = ", ".join(f"{pos}:{cnt}" for pos, cnt in counts.items())
        self.log.debug(f"📊 Raw detections ({len(detected)}): {entries}")
        
    def _is_globally_collected(self, label: str) -> bool:
        """Devuelve True si el PoI ya fue colectado por algún VQC en campo."""
        return getattr(config, "USE_GLOBAL_COLLECTION_LOCK", False) and (label in config.COLLECTED_LABELS)

    def _prune_pending_by_global_lock(self) -> None:
        """Saca de self.pending los PoIs que ya estén bloqueados globalmente."""
        if not getattr(config, "USE_GLOBAL_COLLECTION_LOCK", False):
            return
        before = len(self.pending)
        if before == 0:
            return
        self.pending = [p for p in self.pending if p["label"] not in config.COLLECTED_LABELS]
        removed = before - len(self.pending)
        if removed > 0:
            self.log.debug(f"🧹 Pruned {removed} pending by global lock")


#################################################################################################
# ---------- helpers de tabla/analítica (fuera de la clase) ----------
def _fmt_table(rows, headers=("Métrica", "Valor", "Notas")) -> str:
    if not rows: 
        rows = []
    cols = list(zip(*([headers] + rows))) if rows else [headers]
    widths = [max(len(str(x)) for x in col) for col in cols]
    def line(cells):
        return " | ".join(str(c).ljust(w) for c, w in zip(cells, widths))
    sep = "-+-".join("-" * w for w in widths)
    out = [line(headers), sep]
    out += [line(r) for r in rows]
    return "\n".join(out)

def _mean(vals):
    return (sum(vals) / len(vals)) if vals else float('nan')

def _p95(vals):
    if not vals: return float('nan')
    vs = sorted(vals)
    return vs[int(0.95 * (len(vs) - 1))]

def emit_tables_and_glossary(eqc):
    """
    NO toca tu lógica: solo imprime tabla local, análisis breve
    y cuando termina el último EQC, tabla global + glosario.
    Usa lo que YA calculaste y acumulaste en finish().
    """
    import config
    log = eqc.log

    # ---- LOCAL (por este EQC) ----
    Ls   = [x for _, x in eqc.lat_service]
    Lc   = [x for _, x in eqc.lat_contact]
    Le2e = [x for _, x in eqc.lat_e2e]

    total_time     = eqc.provider.current_time() - eqc.start_time
    unique         = len(config.METRICS.get("unique_ids", []))
    redundant      = config.METRICS.get("redundant", 0)
    assigns        = eqc.assign_count
    success        = eqc.assign_success
    success_rate   = (success/assigns) if assigns>0 else float('nan')
    discovery_rate = (unique/total_time) if total_time>0 else float('nan')

    rows_local = [
        ("Unique PoIs (global)",       unique,                    "PoIs únicos entregados"),
        ("Redundant reports (global)", redundant,                 "Reportes duplicados"),
        ("Assigns sent",               assigns,                   "ASSIGN de este EQC"),
        ("Successful delivers",        success,                   "DELIVER de estas assigns"),
        ("Success rate",               f"{success_rate:.2f}",     "successful/assigns"),
        ("Avg service latency (s)",    f"{_mean(Ls):.3f}",        "t_arrive - t_detect"),
        ("p95 service latency (s)",    f"{_p95(Ls):.3f}",         ""),
        ("Avg contact overhead (s)",   f"{_mean(Lc):.3f}",        "t_deliver_ack - t_arrive"),
        ("p95 contact overhead (s)",   f"{_p95(Lc):.3f}",         ""),
        ("Avg e2e latency (s)",        f"{_mean(Le2e):.3f}",      "t_deliver_ack - t_spawn"),
        ("Detection rate (PoIs/s)",    f"{discovery_rate:.3f}",   "unique / tiempo sim."),
        ("Camera detections (raw)",    eqc.cam_raw_count,         "nodos vistos por cámara"),
        ("Camera PoI matches",         eqc.cam_poi_matches,       "detecciones que eran PoI"),
        ("Global mission score (+)",   f"{eqc.global_score:.2f}", "ponderado por urgencia"),
    ]

    log.info("==== EQC LOCAL SUMMARY ====\n" + _fmt_table(rows_local))

    # análisis local cortito
    if not (assigns > 0 and success > 0):
        log.info("Nota: pocas asignaciones exitosas aún; deja correr más tiempo si buscas latencias estables.")
    elif success_rate < 0.6:
        log.info("⚠️  Success rate bajo: VQC saturados o lejos del líder.")
    elif success_rate > 0.9 and _mean(Ls) < 5.0:
        log.info("✅ Buen aprovechamiento: alta conversión y latencia baja.")
    if _mean(Lc) == _mean(Lc) and _mean(Ls) == _mean(Ls) and _mean(Lc) > 0.3 * _mean(Ls):
        log.info("ℹ️ Contact overhead apreciable respecto a L_service.")

    # ---- GLOBAL (solo cuando termina el último EQC) ----
    m = config.METRICS
    m["eqc_finished"] = m.get("eqc_finished", 0) + 1
    if m["eqc_finished"] >= getattr(config, "NUM_EQCS", 1):
        Ls_all   = m.get("lat_service_all", [])
        Lc_all   = m.get("lat_contact_all", [])
        Le2e_all = m.get("lat_e2e_all", [])
        rows_global = [
            ("Unique PoIs (global)",       len(m.get("unique_ids", [])), ""),
            ("Redundant reports (global)", m.get("redundant", 0),         ""),
            ("Assigns sent (sum EQCs)",    sum(r["assigns"] for r in m.get("eqc_reports", [])), ""),
            ("Successful delivers (sum)",  sum(r["success"] for r in m.get("eqc_reports", [])), ""),
            ("Avg service latency (s)",    f"{_mean(Ls_all):.3f}", "t_arrive - t_detect"),
            ("p95 service latency (s)",    f"{_p95(Ls_all):.3f}",  ""),
            ("Avg contact overhead (s)",   f"{_mean(Lc_all):.3f}", "t_deliver_ack - t_arrive"),
            ("p95 contact overhead (s)",   f"{_p95(Lc_all):.3f}",  ""),
            ("Avg e2e latency (s)",        f"{_mean(Le2e_all):.3f}","t_deliver_ack - t_spawn"),
            ("Camera detections (sum)",    m.get("cam_raw_all",0),  ""),
            ("Camera PoI matches (sum)",   m.get("cam_hits_all",0), ""),
            ("Global mission score (sum)", f"{m.get('global_score',0.0):.2f}", ""),
        ]
        log.info("\n==== GLOBAL SUMMARY (ALL EQCs) ====\n" + _fmt_table(rows_global))

        if Ls_all and _mean(Lc_all) == _mean(Lc_all) and _mean(Ls_all) == _mean(Ls_all) and _mean(Lc_all) > 0.3 * _mean(Ls_all):
            log.info("🌐 GLOBAL: el overhead de contacto es una fracción importante de L_service.")
        if len(m.get("unique_ids", [])) < 0.8 * len(getattr(config, "POIS", [])):
            log.info("🌐 GLOBAL: cobertura moderada; prueba subir K, R_COMM o velocidad.")

        log.info("""
==== GLOSARIO DE MÉTRICAS ====
• Unique PoIs: PoIs distintos entregados al menos una vez.
• Redundant reports: entregas duplicadas del mismo PoI.
• Assigns sent / Successful delivers: ASSIGN enviados / DELIVER recibidos de esas assigns.
• Success rate = successful / assigns.
• Service latency = t_arrive - t_detect (detección EQC → llegada VQC).
• Contact overhead = t_deliver_ack - t_arrive (llegada → ACK).
• e2e latency = t_deliver_ack - t_spawn (inicio del experimento → ACK).
• Detection rate = unique / tiempo_simulación (PoIs por segundo).
• Global mission score = suma de pesos por urgencia (w1<w2<w3) de PoIs únicos.
""")
