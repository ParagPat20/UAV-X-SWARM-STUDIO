#!/usr/bin/env python3
"""
UAV-X 3D Swarm Studio — Ground Control Station Backend
Streams real-time multi-UAV telemetry, vehicle messages (STATUSTEXT),
flight phases, and command feedback over WebSockets.
"""

import os
import sys
import time
import math
import json
import socket
import asyncio
import threading
import http.server
import socketserver
import urllib.parse
from pymavlink import mavutil
for venv_path in [
    os.path.expanduser("~/venv-ardupilot/lib/python3.10/site-packages"),
    os.path.expanduser("~/venv-ardupilot/lib/python3.11/site-packages"),
    os.path.expanduser("~/venv-ardupilot/lib/python3.12/site-packages"),
]:
    if os.path.exists(venv_path) and venv_path not in sys.path:
        sys.path.insert(0, venv_path)

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import websockets

try:
    from rf_model import RFModel
except ImportError:
    from swarm_studio.rf_model import RFModel

try:
    import airsim
except ImportError:
    airsim = None

HTTP_PORT = 8080
WS_PORT = 8765
STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")

DRONE_COLORS = [
    "#00f5d4",  # D1: Cyan
    "#a855f7",  # D2: Purple
    "#ff007f",  # D3: Pink
    "#ffb703",  # D4: Gold
    "#00e676",  # D5: Emerald
    "#3a86ff",  # D6: Royal Blue
    "#fb5607",  # D7: Flame Orange
    "#8338ec",  # D8: Deep Violet
    "#ff006e",  # D9: Hot Coral
    "#00bbf9"   # D10: Sky Blue
]

MODE_MAPPING = {
    0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED",
    5: "LOITER", 6: "RTL", 7: "CIRCLE", 9: "LAND", 16: "POSHOLD", 17: "BRAKE"
}
MODE_STR_TO_NUM = {v: k for k, v in MODE_MAPPING.items()}


class SwarmTelemetryManager:
    """Manages direct MAVLink telemetry threads and command dispatch for swarm drones."""

    def __init__(self, max_drones=10):
        self.max_drones = max_drones
        self.drones = {}
        self.origin = None  # (lat, lon, alt)
        self.pending_missions = {}
        self.pending_auto_start = {}
        self.lock = threading.Lock()
        self.airsim_lock = threading.Lock()
        self.connected_ws = set()
        self.running = True
        self.airsim_client = None
        
        # --- Autonomous Guided Reactive Survey Engine ---
        self.guided_survey_active = False
        self.guided_survey_state = {} # drone_id -> { "waypoints": [...], "wp_idx": 0, "alt": drone_alt, "lane_k": k }
        
        # --- Persistent Settings Engine (swarm_settings.json) ---
        base_dir = os.path.dirname(os.path.abspath(__file__))
        self.settings_file = os.path.join(base_dir, "swarm_settings.json")
        self.geofence_enabled = True
        self.geofence_radius = 85.0
        self.geofence_alt_max = 25.0
        self.avoid_margin = 4.0
        self.swarm_repel = 16.0
        self.min_drone_dist = 3.0
        self.solidify_rate = 1.5
        self.spawn_num_drones = 5
        self.load_settings()

        # --- Downed Drone SAR & Swarm Reconfiguration Engine ---
        self.downed_drones = {}  # drone_id -> { id, x, y, z, lat, lon, timestamp, assigned_rescuer }

        # --- RF Communication Link Topology Engine ---
        self.rf_model = RFModel(gcs_pos=(0, -100, 0), max_range=150.0)

        self._init_airsim_client()
        threading.Thread(target=self._guided_reactive_survey_loop, daemon=True).start()
        threading.Thread(target=self._geofence_monitor_loop, daemon=True).start()
        threading.Thread(target=self._battery_simulation_loop, daemon=True).start()

    def load_settings(self):
        """Loads parameters locally from swarm_settings.json."""
        defaults = {
            "geofence_enabled": True,
            "geofence_radius": 85.0,
            "geofence_alt_max": 25.0,
            "avoid_margin": 4.0,
            "swarm_repel": 16.0,
            "min_drone_dist": 3.0,
            "solidify_rate": 1.5,
            "spawn_num_drones": 5
        }
        if os.path.exists(self.settings_file):
            try:
                with open(self.settings_file, "r") as f:
                    data = json.load(f)
                defaults.update(data)
                print(f"[SETTINGS] Loaded persistent configuration: {defaults}")
            except Exception as e:
                print(f"[SETTINGS] Warning loading {self.settings_file}: {e}")
        self.geofence_enabled = bool(defaults["geofence_enabled"])
        self.geofence_radius = float(defaults["geofence_radius"])
        self.geofence_alt_max = float(defaults["geofence_alt_max"])
        self.avoid_margin = float(defaults["avoid_margin"])
        self.swarm_repel = float(defaults["swarm_repel"])
        self.min_drone_dist = float(defaults["min_drone_dist"])
        self.solidify_rate = float(defaults["solidify_rate"])
        self.spawn_num_drones = int(defaults["spawn_num_drones"])

    def save_settings(self):
        """Saves current parameters locally to swarm_settings.json."""
        data = {
            "geofence_enabled": self.geofence_enabled,
            "geofence_radius": self.geofence_radius,
            "geofence_alt_max": self.geofence_alt_max,
            "avoid_margin": self.avoid_margin,
            "swarm_repel": self.swarm_repel,
            "min_drone_dist": self.min_drone_dist,
            "solidify_rate": self.solidify_rate,
            "spawn_num_drones": self.spawn_num_drones
        }
        try:
            with open(self.settings_file, "w") as f:
                json.dump(data, f, indent=2)
            print(f"[SETTINGS] Saved parameters locally to {self.settings_file}")
        except Exception as e:
            print(f"[SETTINGS] Error saving {self.settings_file}: {e}")

    def _init_airsim_client(self):
        if airsim:
            with self.airsim_lock:
                try:
                    self.airsim_client = airsim.MultirotorClient()
                    self.airsim_client.confirmConnection()
                    print("★ Connected to Microsoft AirSim RPC Server (Port 41451)")
                except Exception:
                    self.airsim_client = None

    def init_drones(self):
        for i in range(1, self.max_drones + 1):
            port = 5762 + (i - 1) * 10
            color = DRONE_COLORS[(i - 1) % len(DRONE_COLORS)]
            self.drones[i] = {
                "id": i,
                "port": port,
                "conn": None,
                "connected": False,
                "color": color,
                "lat": 0.0,
                "lon": 0.0,
                "alt": 0.0,
                "relative_alt": 0.0,
                "target_alt": 3.0,
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
                "heading": 0.0,
                "vx": 0.0,
                "vy": 0.0,
                "vz": 0.0,
                "speed": 0.0,
                "armed": False,
                "mode": "DISCONNECTED",
                "flight_phase": "STANDBY",
                "battery": 100,
                "voltage": 12.6,
                "current": 0.0,
                "rssi": 98,
                "satellites": 15,
                "fix_type": 3,
                "current_wp": 0,
                "total_wps": 0,
                "last_status_msg": "Standby - Waiting for SITL connection...",
                "status_severity": 6,
                "last_ack": "NONE",
                "last_update": 0,
                "trail": []
            }
            # Start background reader thread for this drone
            t = threading.Thread(target=self._drone_reader_loop, args=(i, port), daemon=True)
            t.start()

    def _drone_reader_loop(self, drone_idx, default_port):
        offset = (drone_idx - 1) * 10
        candidate_ports = [5763 + offset, 5762 + offset, 5760 + offset]
        
        while self.running:
            conn = None
            active_port = None
            try:
                for port in candidate_ports:
                    try:
                        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                        s.settimeout(0.2)
                        s.connect(('127.0.0.1', port))
                        s.close()
                        active_port = port
                        break
                    except Exception:
                        continue

                if not active_port:
                    time.sleep(1.5)
                    continue

                conn_str = f"tcp:127.0.0.1:{active_port}"
                conn = mavutil.mavlink_connection(conn_str)
                hb = conn.wait_heartbeat(timeout=3)
                if not hb:
                    conn.close()
                    time.sleep(1.0)
                    continue

                sysid = hb.get_srcSystem()
                with self.lock:
                    self.drones[drone_idx]["conn"] = conn
                    self.drones[drone_idx]["connected"] = True
                    self.drones[drone_idx]["port"] = active_port
                    self.drones[drone_idx]["last_status_msg"] = f"Connected on port {active_port}"

                # Request data streams
                try:
                    conn.mav.request_data_stream_send(
                        sysid, 1,
                        mavutil.mavlink.MAV_DATA_STREAM_ALL,
                        25, 1
                    )
                except Exception:
                    pass

                last_hb = time.time()
                while self.running:
                    msg = conn.recv_msg()
                    if msg is None:
                        if time.time() - last_hb > 5.0:
                            break
                        time.sleep(0.01)
                        continue

                    if msg.get_srcSystem() != sysid:
                        continue

                    mtype = msg.get_type()
                    t_now = time.time()

                    with self.lock:
                        d = self.drones[drone_idx]
                        d["last_update"] = t_now

                        if mtype == 'HEARTBEAT':
                            last_hb = t_now
                            d["armed"] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                            d["mode"] = MODE_MAPPING.get(msg.custom_mode, f"MODE_{msg.custom_mode}")
                            d["connected"] = True

                        elif mtype == 'GLOBAL_POSITION_INT':
                            lat = msg.lat / 1e7
                            lon = msg.lon / 1e7
                            alt = msg.relative_alt / 1000.0
                            hdg = msg.hdg / 100.0
                            vx = msg.vx / 100.0
                            vy = msg.vy / 100.0
                            vz = msg.vz / 100.0

                            if self.origin is None and lat != 0.0 and lon != 0.0:
                                self.origin = (lat, lon, 0.0)

                            if self.origin is not None:
                                o_lat, o_lon, _ = self.origin
                                x = (lon - o_lon) * (111320.0 * math.cos(math.radians(o_lat)))
                                y = (lat - o_lat) * 111320.0
                                z = max(0.0, alt)
                                d["x"] = round(x, 3)
                                d["y"] = round(z, 3)
                                d["z"] = round(-y, 3)
                            
                            d["lat"] = round(lat, 7)
                            d["lon"] = round(lon, 7)
                            d["alt"] = round(alt, 2)
                            d["relative_alt"] = round(alt, 2)
                            d["heading"] = round(hdg, 1)
                            d["vx"] = round(vx, 2)
                            d["vy"] = round(vy, 2)
                            d["vz"] = round(vz, 2)
                            d["speed"] = round(math.sqrt(vx*vx + vy*vy), 2)

                            # Record trailing point for 3D ribbon tail in active flight
                            if d["armed"] and d["alt"] > 0.4 and self.origin is not None:
                                trail = d["trail"]
                                pt = [d["x"], d["y"], d["z"]]
                                if pt != [0, 0, 0] and pt[1] > 0.3:
                                    if not trail:
                                        trail.append(pt)
                                    elif 0.3 < math.dist(trail[-1], pt) < 25.0:
                                        trail.append(pt)
                                        if len(trail) > 150:
                                            trail.pop(0)

                        elif mtype == 'ATTITUDE':
                            d["roll"] = round(math.degrees(msg.roll), 1)
                            d["pitch"] = round(math.degrees(msg.pitch), 1)
                            d["yaw"] = round(math.degrees(msg.yaw) % 360, 1)

                        elif mtype == 'STATUSTEXT':
                            raw_text = msg.text.strip()
                            d["last_status_msg"] = raw_text
                            d["status_severity"] = msg.severity
                            # Append to message history
                            t_str = time.strftime("%H:%M:%S")
                            msg_entry = {
                                "time": t_str,
                                "text": raw_text,
                                "severity": msg.severity,
                                "is_error": msg.severity <= 4
                            }
                            if "msg_history" not in d:
                                d["msg_history"] = []
                            d["msg_history"].append(msg_entry)
                            if len(d["msg_history"]) > 6:
                                d["msg_history"].pop(0)

                        elif mtype == 'COMMAND_ACK':
                            res_names = {0: "ACCEPTED", 1: "TEMPORARILY_REJECTED", 2: "DENIED", 3: "UNSUPPORTED", 4: "FAILED", 5: "IN_PROGRESS"}
                            res_str = res_names.get(msg.result, str(msg.result))
                            d["last_ack"] = f"ACK: {res_str}"
                            if msg.result != 0:
                                if not d.get("ekf_ready", False) or d.get("alt", 0.0) < 0.5:
                                    d["last_status_msg"] = "⏳ Calibrating EKF/Sensors..."
                                    d["status_severity"] = 6
                                else:
                                    d["last_status_msg"] = f"Command {res_str}"
                                    d["status_severity"] = 4  # Warning

                        elif mtype == 'SYS_STATUS':
                            # Battery is simulated in _battery_simulation_loop
                            # if msg.battery_remaining != -1:
                            #     d["battery"] = max(0, min(100, msg.battery_remaining))
                            if msg.voltage_battery > 0:
                                d["voltage"] = round(msg.voltage_battery / 1000.0, 2)
                            if msg.current_battery > 0:
                                d["current"] = round(msg.current_battery / 100.0, 2)

                        elif mtype == 'EKF_STATUS_REPORT':
                            # flags: 1=ATTITUDE, 2=HORIZ_VEL, 4=VERT_VEL, 8=HORIZ_POS_REL, 16=HORIZ_POS_ABS, 32=VERT_POS
                            d["ekf_flags"] = msg.flags
                            d["ekf_ready"] = bool(msg.flags & 16) # True when horizontal absolute position is valid

                        elif mtype == 'GPS_RAW_INT':
                            d["satellites"] = msg.satellites_visible
                            d["fix_type"] = msg.fix_type

                        elif mtype in ('MISSION_REQUEST', 'MISSION_REQUEST_INT'):
                            req_seq = msg.seq
                            if drone_idx in self.pending_missions:
                                wps = self.pending_missions[drone_idx]
                                if req_seq < len(wps):
                                    item = wps[req_seq]
                                    conn.mav.mission_item_int_send(
                                        sysid, 1,
                                        req_seq,
                                        item.get('frame', mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT),
                                        item.get('command', mavutil.mavlink.MAV_CMD_NAV_WAYPOINT),
                                        item.get('current', 1 if req_seq == 0 else 0),
                                        item.get('autocontinue', 1),
                                        item.get('param1', 0), item.get('param2', 0),
                                        item.get('param3', 0), item.get('param4', 0),
                                        int(item['lat'] * 1e7),
                                        int(item['lon'] * 1e7),
                                        float(item['alt']),
                                        0
                                    )

                        elif mtype == 'MISSION_ACK':
                            if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                                d["last_status_msg"] = "Mission Upload Accepted ✓"
                                conn.mav.mission_set_current_send(sysid, 1, 1)
                                if self.pending_auto_start.get(drone_idx, False):
                                    conn.mav.command_long_send(sysid, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
                                    conn.mav.command_long_send(sysid, 1, mavutil.mavlink.MAV_CMD_MISSION_START, 0, 1, 0, 0, 0, 0, 0, 0)
                                    conn.set_mode(3) # AUTO
                                    d["last_status_msg"] = "AUTO Survey Started 🚀"
                            else:
                                d["last_status_msg"] = f"Mission Ack: {msg.type}"

                        elif mtype == 'MISSION_CURRENT':
                            d["current_wp"] = msg.seq

                        # Compute dynamic flight phase (in NED: vz < 0 is climbing, vz > 0 is descending)
                        climb_rate = -d["vz"]
                        if not d["armed"]:
                            if "Need Position Estimate" in d.get("last_status_msg", ""):
                                d["flight_phase"] = "ACQUIRING POSITION ESTIMATE..."
                            elif d["lat"] != 0.0 and d.get("fix_type", 0) >= 3 and d.get("ekf_ready", True):
                                d["flight_phase"] = "READY (STANDBY)"
                            else:
                                d["flight_phase"] = "INITIALIZING EKF..."
                        elif d["alt"] < 0.4 and d["mode"] == "GUIDED":
                            d["flight_phase"] = "ARMED (ON GROUND)"
                        elif d["mode"] == "LAND":
                            d["flight_phase"] = "LANDING"
                        elif d["mode"] == "RTL":
                            d["flight_phase"] = "RETURNING TO LAUNCH (RTL)"
                        elif d["mode"] == "AUTO":
                            d["flight_phase"] = f"AUTO SURVEY (WP {d['current_wp']})"
                        elif climb_rate > 0.3 and d["alt"] < d.get("target_alt", 3.0) - 0.3:
                            d["flight_phase"] = f"CLIMBING TO {d.get('target_alt', 3.0):.1f}m ({climb_rate:+.1f}m/s)"
                        elif climb_rate < -0.3 and d["alt"] > 0.5:
                            d["flight_phase"] = f"DESCENDING ({climb_rate:+.1f}m/s)"
                        elif d["alt"] >= 0.4:
                            d["flight_phase"] = f"HOVERING / FLYING ({d['alt']:.1f}m)"
                        else:
                            d["flight_phase"] = "STANDBY"

            except Exception:
                pass
            finally:
                with self.lock:
                    self.drones[drone_idx]["connected"] = False
                    self.drones[drone_idx]["conn"] = None
                if conn:
                    try:
                        conn.close()
                    except Exception:
                        pass
                time.sleep(2.0)

    def get_airsim_client(self):
        """Thread-safe persistent connection to AirSim RPC server."""
        if not airsim:
            return None
        with self.airsim_lock:
            if self.airsim_client is None:
                try:
                    c = airsim.MultirotorClient()
                    c.confirmConnection()
                    self.airsim_client = c
                except Exception:
                    self.airsim_client = None
            return self.airsim_client

    def fetch_camera_frame(self, drone_idx, cam_name="0", image_type=0):
        """Fetches live camera frame PNG bytes from AirSim for UAV{drone_idx}."""
        client = self.get_airsim_client()
        if not client or not airsim:
            return None
        try:
            uav_name = f"UAV{drone_idx}"
            with self.airsim_lock:
                req_cam = str(cam_name)
                req_type = airsim.ImageType(image_type)
                # compress=True returns PNG-compressed image bytes
                responses = client.simGetImages([
                    airsim.ImageRequest(req_cam, req_type, False, True)
                ], vehicle_name=uav_name)
            if responses and len(responses) > 0 and len(responses[0].image_data_uint8) > 0:
                return bytes(responses[0].image_data_uint8)
        except Exception:
            pass
        return None

    def fetch_airsim_lidar_hits(self, drone_idx, d):
        """Fetches real 360 LiDAR point cloud hits from Microsoft AirSim environment."""
        client = self.get_airsim_client()
        if not client:
            return None, 99.0
        try:
            uav_name = f"UAV{drone_idx}"
            with self.airsim_lock:
                try:
                    lidar_data = client.getLidarData(lidar_name="lidar", vehicle_name=uav_name)
                except Exception:
                    lidar_data = None
                if not lidar_data or len(lidar_data.point_cloud) < 3:
                    try:
                        lidar_data = client.getLidarData(lidar_name="Lidar360", vehicle_name=uav_name)
                    except Exception:
                        lidar_data = None
            if not lidar_data or len(lidar_data.point_cloud) < 3:
                return None, 99.0

            pts = lidar_data.point_cloud
            hits = []
            min_d = 99.0
            px, py, pz = d["x"], d["y"], d["z"]

            hdg_rad = math.radians(d.get("heading", 0.0))
            pitch_rad = math.radians(d.get("pitch", 0.0))
            roll_rad = math.radians(d.get("roll", 0.0))

            cos_psi, sin_psi = math.cos(hdg_rad), math.sin(hdg_rad)
            cos_th, sin_th = math.cos(pitch_rad), math.sin(pitch_rad)
            cos_phi, sin_phi = math.cos(roll_rad), math.sin(roll_rad)

            total_pts = len(pts) // 3
            sample_step = max(1, total_pts // 72)

            for i in range(0, total_pts, sample_step):
                idx = i * 3
                lx = pts[idx]     # Forward (+X)
                ly = pts[idx + 1] # Right (+Y)
                lz = pts[idx + 2] # Down (+Z)

                r_dist = math.sqrt(lx * lx + ly * ly + lz * lz)
                # Ignore self-drone reflections (<0.4m) and record all valid environmental returns
                if r_dist > 0.4:
                    # 3D rotation from sensor body frame to world NED (North, East, Down)
                    v_n = lx * cos_th * cos_psi + ly * (sin_phi * sin_th * cos_psi - cos_phi * sin_psi) + lz * (cos_phi * sin_th * cos_psi + sin_phi * sin_psi)
                    v_e = lx * cos_th * sin_psi + ly * (sin_phi * sin_th * sin_psi + cos_phi * cos_psi) + lz * (cos_phi * sin_th * sin_psi - sin_phi * cos_psi)
                    v_d = -lx * sin_th + ly * sin_phi * cos_th + lz * cos_phi * cos_th

                    # In Three.js: +X = East (v_e), +Y = Up (-v_d), +Z = South (-v_n)
                    hx = round(px + v_e, 3)
                    hy = round(max(0.0, py - v_d), 3)
                    hz = round(pz - v_n, 3)
                    deg = round((math.degrees(math.atan2(ly, lx)) + 360) % 360)

                    # Filter out road / ground surface hits (hy <= 0.45m)
                    if hy <= 0.45:
                        continue

                    hits.append([hx, hy, hz, round(r_dist, 2), deg])
                    if r_dist < min_d:
                        min_d = round(r_dist, 2)

            if hits:
                return hits, min_d
        except Exception:
            self.airsim_client = None  # Force reconnection on error
        return None, 99.0

    def compute_lidar_360_hits(self, d):
        """Computes 360-degree LiDAR rangefinder hits against AirSim obstacles."""
        if not d["connected"]:
            return [], 99.0

        drone_idx = d["id"]

        # 1. Primary Source: Live AirSim LiDAR Point Cloud
        airsim_hits, airsim_min_dist = self.fetch_airsim_lidar_hits(drone_idx, d)
        if airsim_hits is not None:
            return airsim_hits, airsim_min_dist

        # 2. Secondary Source: MAVLink Distance Sensors
        px, py, pz = d["x"], d["y"], d["z"]
        heading_rad = math.radians(d.get("heading", 0.0))
        hits = []
        min_dist = 99.0

        mav_distances = d.get("mav_distance_sensors", {})
        if mav_distances:
            for orient, (dist_m, orient_deg) in mav_distances.items():
                if dist_m > 0.2:
                    theta = heading_rad + math.radians(orient_deg)
                    hx = round(px + dist_m * math.sin(theta), 2)
                    hy = round(py, 2)
                    hz = round(pz - dist_m * math.cos(theta), 2)
                    hits.append([hx, hy, hz, dist_m, round(orient_deg)])
                    if dist_m < min_dist:
                        min_dist = dist_m

        return hits, min_dist

    def get_snapshot(self):
        """Build telemetry payload for frontend WebSocket streaming."""
        with self.lock:
            active_drones = [d for d in self.drones.values() if d["connected"]]

            total_active = len(active_drones)
            avg_alt = round(sum(d["alt"] for d in active_drones) / max(1, total_active), 1) if active_drones else 0.0
            avg_battery = int(sum(d["battery"] for d in active_drones) / max(1, total_active)) if active_drones else 100
            max_speed = round(max((d["speed"] for d in active_drones), default=0.0), 1)
            all_armed = all(d["armed"] for d in active_drones) if active_drones else False

            topology = self.rf_model.compute_topology(active_drones)

            serialized_drones = {}
            for d_id, d in self.drones.items():
                if d["connected"]:
                    lidar_hits, min_obs_dist = self.compute_lidar_360_hits(d)
                    
                    # Update dynamic RF metrics
                    rf_info = topology.get(d_id, {'pdr': 0, 'rssi': 0, 'hops': [], 'parent_id': None})
                    d["rssi"] = rf_info['rssi']
                    d["pdr"] = round(rf_info['pdr'] * 100, 1)

                    serialized_drones[d_id] = {
                        "id": d["id"],
                        "color": d["color"],
                        "connected": d["connected"],
                        "armed": d["armed"],
                        "mode": d["mode"],
                        "flight_phase": d["flight_phase"],
                        "last_status_msg": d["last_status_msg"],
                        "status_severity": d["status_severity"],
                        "last_ack": d["last_ack"],
                        "lat": d["lat"],
                        "lon": d["lon"],
                        "alt": d["alt"],
                        "relative_alt": d["relative_alt"],
                        "target_alt": d["target_alt"],
                        "x": d["x"],
                        "y": d["y"],
                        "z": d["z"],
                        "roll": d["roll"],
                        "pitch": d["pitch"],
                        "yaw": d["yaw"],
                        "heading": d["heading"],
                        "speed": d["speed"],
                        "battery": d["battery"],
                        "voltage": d["voltage"],
                        "current": d["current"],
                        "rssi": d["rssi"],
                        "pdr": d["pdr"],
                        "hops": rf_info["hops"],
                        "parent_id": rf_info["parent_id"],
                        "satellites": d["satellites"],
                        "fix_type": d["fix_type"],
                        "current_wp": d["current_wp"],
                        "lidar_hits": lidar_hits,
                        "min_obstacle_dist": min_obs_dist if min_obs_dist < 90 else None,
                        "msg_history": d.get("msg_history", []),
                        "trail": d["trail"][-60:],
                        "predicted_trajectory": d.get("predicted_trajectory", []),
                        "survey_waypoints": d.get("survey_waypoints", []),
                        "geofence_breach": d.get("geofence_breach", False),
                        "low_battery_rtb": d.get("low_battery_rtb_triggered", False)
                    }

            return {
                "timestamp": time.time(),
                "swarm": {
                    "active_count": total_active,
                    "avg_alt": avg_alt,
                    "avg_battery": avg_battery,
                    "max_speed": max_speed,
                    "all_armed": all_armed,
                    "origin": self.origin,
                    "geofence": {
                        "enabled": self.geofence_enabled,
                        "radius": self.geofence_radius,
                        "alt_max": self.geofence_alt_max,
                        "center": [0.0, 0.0, 0.0]
                    },
                    "gcs_pos": self.rf_model.gcs_pos
                },
                "settings": {
                    "geofence_enabled": self.geofence_enabled,
                    "geofence_radius": self.geofence_radius,
                    "geofence_alt_max": self.geofence_alt_max,
                    "avoid_margin": self.avoid_margin,
                    "swarm_repel": self.swarm_repel,
                    "min_drone_dist": self.min_drone_dist,
                    "solidify_rate": self.solidify_rate,
                    "spawn_num_drones": self.spawn_num_drones
                },
                "downed_drones": list(self.downed_drones.values()),
                "drones": serialized_drones
            }

    def _battery_simulation_loop(self):
        """1Hz Autonomous Battery Simulation & SoC Management."""
        while self.running:
            try:
                with self.lock:
                    for sysid, d in self.drones.items():
                        if not d.get("connected") or not d.get("armed"):
                            continue
                        
                        # SoC simulation: SoC(t) = SoC(0) - ∫(I_base + k·v²) dt
                        # Hover current drain base = 1.5% per second (Accelerated for demonstration)
                        # Velocity penalty k = 0.002
                        i_base = 1.5
                        k_vel = 0.002
                        v_sq = d.get("vx", 0)**2 + d.get("vy", 0)**2 + d.get("vz", 0)**2
                        depletion = i_base + k_vel * v_sq
                        
                        d["battery"] = max(0.0, d.get("battery", 100.0) - depletion)
                        
                        # RTB Trigger at 25%
                        if d["battery"] <= 25.0 and not d.get("low_battery_rtb_triggered", False):
                            d["low_battery_rtb_triggered"] = True
                            d["last_status_msg"] = "⚠️ LOW BATTERY - AUTO RTL"
                            # Trigger RTL MAVLink command
                            if d.get("conn"):
                                try:
                                    d["conn"].mav.command_long_send(
                                        d["conn"].target_system, d["conn"].target_component,
                                        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
                                        0, 0, 0, 0, 0, 0, 0, 0
                                    )
                                except Exception:
                                    pass
            except Exception:
                pass
            time.sleep(1.0)

    def _geofence_monitor_loop(self):
        """10Hz Autonomous Geofence Boundary Telemetry & Monitoring."""
        while self.running:
            try:
                if self.geofence_enabled:
                    with self.lock:
                        for sysid, d in self.drones.items():
                            if not d.get("connected") or not d.get("conn") or not d.get("armed") or d.get("alt", 0) < 0.8 or d.get("lat") == 0.0:
                                d["geofence_breach"] = False
                                continue
                            px, py, pz = d["x"], d["y"], d["z"]
                            dist_r = math.sqrt(px * px + pz * pz)
                            alt = d["alt"]

                            # Breach check (radius or ceiling exceeded while flying)
                            is_breached = (dist_r >= self.geofence_radius or alt >= self.geofence_alt_max)
                            d["geofence_breach"] = is_breached

                            if is_breached:
                                d["last_status_msg"] = f"⚠️ GEOFENCE LIMIT: {dist_r:.1f}m / {self.geofence_radius:.0f}m"
                                if not self.guided_survey_active:
                                    d["flight_phase"] = "GEOFENCE BOUNDARY"
            except Exception:
                pass
            time.sleep(0.1)

    def _guided_reactive_survey_loop(self):
        """10Hz Real-Time Autonomous Swarm Elastic Geofence Frontier Exploration & Reactive SLAM Controller."""
        while self.running:
            if not self.guided_survey_active or not self.guided_survey_state:
                time.sleep(0.1)
                continue

            try:
                with self.lock:
                    # Dynamic Real-Time Nearest Rescuer Re-Evaluation for all active downed drones
                    for downed_id, d_info in list(self.downed_drones.items()):
                        if d_info.get("sar_resolved"):
                            continue
                        downed_x, downed_z = d_info["x"], d_info["z"]
                        best_rescuer = None
                        min_dist = 999999.0
                        for other_id, other_d in self.drones.items():
                            if other_id != downed_id and other_d.get("connected") and other_d.get("armed") and not other_d.get("failed") and other_d.get("alt", 0) > 0.8:
                                dist_to_downed = math.sqrt((other_d["x"] - downed_x)**2 + (other_d["z"] - downed_z)**2)
                                if dist_to_downed < min_dist:
                                    min_dist = dist_to_downed
                                    best_rescuer = other_id

                        current_rescuer = d_info.get("assigned_rescuer")
                        if best_rescuer is not None:
                            if current_rescuer is None or current_rescuer != best_rescuer:
                                if current_rescuer is not None and current_rescuer in self.drones:
                                    cur_dist = math.sqrt((self.drones[current_rescuer]["x"] - downed_x)**2 + (self.drones[current_rescuer]["z"] - downed_z)**2)
                                    # Switch rescuer if the new drone is closer by at least 2.0m (hysteresis) or if old rescuer failed/landed
                                    old_drone = self.drones[current_rescuer]
                                    if min_dist < cur_dist - 2.0 or not old_drone.get("armed") or old_drone.get("failed"):
                                        old_drone["sar_arrived"] = False
                                        old_drone["sar_downed_id"] = None
                                        d_info["assigned_rescuer"] = best_rescuer
                                        print(f"[DYNAMIC_SAR] Rescuer dynamically switched for UAV {downed_id}: UAV {current_rescuer} ({cur_dist:.1f}m) -> UAV {best_rescuer} ({min_dist:.1f}m)")
                                else:
                                    d_info["assigned_rescuer"] = best_rescuer

                    active_ids = list(self.guided_survey_state.keys())

                for sysid in active_ids:
                    d = self.drones.get(sysid)
                    s = self.guided_survey_state.get(sysid)
                    if not d or not s or not d["conn"] or not d["connected"]:
                        continue

                    conn = d["conn"]
                    target_alt = s.get("alt", 6.0)
                    primary_sector_deg = s.get("primary_sector_deg", 0.0)
                    explore_radius = min(s.get("explore_radius", 60.0), self.geofence_radius - 2.0 if self.geofence_enabled else 60.0)
                    turn_dir = s.get("turn_dir", 1.0)
                    center_x = s.get("center_x", 0.0)
                    center_z = s.get("center_z", 0.0)

                    # 1. Automatic Arm & Takeoff handling with EKF readiness check & rate limiting
                    now_ts = time.time()
                    if not d["armed"] or d["alt"] < 0.8:
                        if not d.get("ekf_ready", True) or d["lat"] == 0.0:
                            d["flight_phase"] = "WAITING FOR EKF..."
                            d["last_status_msg"] = "⏳ Calibrating EKF/GPS lock before launch..."
                            continue

                        last_to = s.get("last_takeoff_cmd_ts", 0.0)
                        if now_ts - last_to > 3.0:
                            s["last_takeoff_cmd_ts"] = now_ts
                            d["flight_phase"] = f"EXPLORER TAKEOFF ({target_alt:.1f}m)"
                            d["last_status_msg"] = f"Taking off to {target_alt:.1f}m for frontier survey..."
                            conn.set_mode(4)  # GUIDED
                            time.sleep(0.05)
                            conn.mav.command_long_send(sysid, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
                            time.sleep(0.05)
                            conn.mav.command_long_send(sysid, 1, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, target_alt)
                        continue

                    px, py, pz = d["x"], d["y"], d["z"]

                    # 2. 360° Polar Horizon & Free Gap Analysis (36 Sectors, 10° each)
                    MIN_OBS_MARGIN = self.avoid_margin  # Configurable obstacle standoff (default 4.0m)
                    MIN_DRONE_DIST = self.min_drone_dist  # Configurable minimum drone separation (default 3.0m)
                    SWARM_REPEL_DIST = self.swarm_repel  # Configurable inter-drone repulsion radius (default 16.0m)

                    lidar_hits, _ = self.compute_lidar_360_hits(d)
                    sector_clearance = [40.0] * 36  # default 40m open horizon
                    min_obs_all = 99.0
                    closest_obs_vec = None

                    if lidar_hits:
                        for hit in lidar_hits[:180]:
                            hx, hy, hz = hit[0], hit[1], hit[2]
                            if abs(hy - py) < 3.5:
                                rox = hx - px
                                roz = hz - pz
                                dist_h = math.sqrt(rox * rox + roz * roz)
                                if dist_h < min_obs_all:
                                    min_obs_all = dist_h
                                    closest_obs_vec = (rox, roz)
                                # Angle in horizontal plane (Three.js: East=+X, South=+Z, 0°=North)
                                ang_deg = (math.degrees(math.atan2(rox, -roz)) + 360) % 360
                                sec_idx = int(ang_deg / 10.0) % 36
                                if dist_h < sector_clearance[sec_idx]:
                                    sector_clearance[sec_idx] = dist_h

                    # 3. Inter-Drone Shared Awareness & Repulsion Field
                    drone_repulsion = [0.0] * 36
                    min_other_drone_dist = 99.0
                    emergency_drone_push = [0.0, 0.0]

                    for other_id, other_d in self.drones.items():
                        if other_id != sysid and other_d.get("connected") and other_d.get("armed"):
                            ox, oz = other_d["x"], other_d["z"]
                            ddist = math.sqrt((ox - px)**2 + (oz - pz)**2)
                            if ddist < min_other_drone_dist:
                                min_other_drone_dist = ddist

                            # Emergency mutual repulsion if below MIN_DRONE_DIST + 0.5m
                            if ddist < (MIN_DRONE_DIST + 0.5) and ddist > 0.01:
                                push_ux = (px - ox) / ddist
                                push_uz = (pz - oz) / ddist
                                push_mag = max(0.5, ((MIN_DRONE_DIST + 0.5) - ddist) * 2.0)
                                emergency_drone_push[0] += push_ux * push_mag
                                emergency_drone_push[1] += push_uz * push_mag

                            # Angular sector repulsion within SWARM_REPEL_DIST
                            if ddist < SWARM_REPEL_DIST:
                                d_ang = (math.degrees(math.atan2(ox - px, -(oz - pz))) + 360) % 360
                                d_idx = int(d_ang / 10.0) % 36
                                for offset in (-3, -2, -1, 0, 1, 2, 3):
                                    idx_w = (d_idx + offset) % 36
                                    pen = ((SWARM_REPEL_DIST - ddist) / SWARM_REPEL_DIST) ** 2 * (4.0 - abs(offset)) * 40.0
                                    drone_repulsion[idx_w] += pen

                    # 4. Search & Rescue (SAR) Mission Reconfiguration Check
                    sar_target = None
                    for downed_id, d_info in self.downed_drones.items():
                        if not d_info.get("sar_resolved") and d_info.get("assigned_rescuer") == sysid:
                            sar_target = d_info
                            break

                    if sar_target is not None:
                        dx_sar = sar_target["x"] - px
                        dz_sar = sar_target["z"] - pz
                        dist_to_crash = math.sqrt(dx_sar * dx_sar + dz_sar * dz_sar)
                        downed_uav_id = sar_target["id"]

                        if dist_to_crash < 4.0:
                            # Rescuer has reached the crash site! STOP & HOLD POS over accidental area
                            d["flight_phase"] = f"HOLDING SAR @ UAV {downed_uav_id} CRASH SITE"
                            d["last_status_msg"] = f"🚨 ARRIVED at UAV {downed_uav_id} crash site ({sar_target['x']:.1f}, {sar_target['z']:.1f}) - Holding SAR position!"
                            d["predicted_trajectory"] = [[round(px, 2), round(target_alt, 2), round(pz, 2)]]
                            d["sar_arrived"] = True
                            d["sar_downed_id"] = downed_uav_id

                            # Command zero horizontal velocity & hold position over crash site
                            if self.origin is not None:
                                o_lat, o_lon, _ = self.origin
                                tgt_lat = o_lat + (-sar_target["z"] / 111320.0)
                                tgt_lon = o_lon + (sar_target["x"] / (111320.0 * math.cos(math.radians(o_lat))))
                                conn.mav.set_position_target_global_int_send(
                                    0, sysid, 1,
                                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                    0b0000111111111000,
                                    int(tgt_lat * 1e7), int(tgt_lon * 1e7), float(target_alt),
                                    0, 0, 0, 0, 0, 0, 0, 0
                                )
                            continue
                        else:
                            # Actively steer directly towards the downed drone coordinates
                            sar_direct_ang = (math.degrees(math.atan2(dx_sar, -dz_sar)) + 360) % 360
                            primary_sector_deg = sar_direct_ang
                            s["primary_sector_deg"] = sar_direct_ang
                            d["flight_phase"] = f"SAR EN-ROUTE -> UAV {downed_uav_id} ({dist_to_crash:.1f}m)"
                            d["last_status_msg"] = f"🚨 En-route to UAV {downed_uav_id} crash site | Dist: {dist_to_crash:.1f}m"
                            d["sar_arrived"] = False

                    # 5. Current Heading & Forward Momentum
                    cur_hdg = d.get("heading", primary_sector_deg)
                    smooth_hdg = s.get("smooth_hdg", cur_hdg)

                    # Check front corridor clearance (cone [-30°, +30°] around current heading)
                    front_clearance = 40.0
                    for off in range(-3, 4):
                        f_idx = int((smooth_hdg + off * 10.0 + 360) / 10.0) % 36
                        front_clearance = min(front_clearance, sector_clearance[f_idx])

                    # 6. Geofence Elastic Containment Vector & Distance Evaluation
                    dist_from_origin = math.sqrt(px * px + pz * pz)
                    fence_limit_r = self.geofence_radius if self.geofence_enabled else explore_radius
                    fence_warn_r = fence_limit_r * 0.70

                    # 7. Target Exploration Goal Direction
                    t_elapsed = now_ts - s.get("start_time", now_ts)
                    wander_drift = math.sin(t_elapsed * 0.04 + sysid * 1.5) * 45.0 if sar_target is None else 0.0
                    target_goal_deg = (primary_sector_deg + wander_drift + 360.0) % 360.0

                    # If approaching perimeter and not in urgent SAR, deflect target goal tangentially
                    if dist_from_origin > fence_warn_r and sar_target is None:
                        radial_out_ang = (math.degrees(math.atan2(px, -pz)) + 360) % 360
                        tangent_ang = (radial_out_ang + (90.0 * turn_dir) + 360) % 360
                        inward_ang = (radial_out_ang + 180.0 + 360) % 360
                        blend_factor = min(1.0, (dist_from_origin - fence_warn_r) / max(1.0, fence_limit_r - fence_warn_r))
                        target_goal_deg = (tangent_ang * (1.0 - blend_factor * 0.5) + inward_ang * (blend_factor * 0.5) + 360) % 360

                    # 7. Obstacle Wall-Following Hysteresis (Lock turn direction when dodging)
                    avoid_side = s.get("avoid_side", 0)  # +1 = Turn Right, -1 = Turn Left, 0 = Clear
                    if front_clearance < (MIN_OBS_MARGIN + 1.5):
                        if avoid_side == 0:
                            left_clearance = sum(sector_clearance[int((smooth_hdg - off * 10 + 360) / 10) % 36] for off in range(3, 8))
                            right_clearance = sum(sector_clearance[int((smooth_hdg + off * 10 + 360) / 10) % 36] for off in range(3, 8))
                            avoid_side = 1 if right_clearance >= left_clearance else -1
                            s["avoid_side"] = avoid_side
                    elif front_clearance > (MIN_OBS_MARGIN + 3.5):
                        avoid_side = 0
                        s["avoid_side"] = 0

                    # 8. Candidate Sectors Evaluation with Strict 4.0m Obstacle & 3.0m Inter-Drone Containment
                    forward_candidates = []
                    for off_deg in range(-90, 95, 10):
                        cand_ang = (smooth_hdg + off_deg + 360.0) % 360.0
                        cand_idx = int(cand_ang / 10.0) % 36
                        clearance = sector_clearance[cand_idx]
                        
                        # Strict rule 1: Must stay >= 4.0m away from obstacles
                        if clearance < MIN_OBS_MARGIN:
                            continue

                        # Predictive projection (2.5s ahead)
                        c_rad = math.radians(cand_ang)
                        proj_dist = 2.4 * 2.5
                        proj_x = px + proj_dist * math.sin(c_rad)
                        proj_z = pz - proj_dist * math.cos(c_rad)

                        # Strict rule 2: Projected path must stay >= 3.0m away from all other drones
                        too_close_to_drone = False
                        for other_id, other_d in self.drones.items():
                            if other_id != sysid and other_d.get("connected") and other_d.get("armed"):
                                d_proj = math.sqrt((proj_x - other_d["x"])**2 + (proj_z - other_d["z"])**2)
                                if d_proj < MIN_DRONE_DIST + 0.5:
                                    too_close_to_drone = True
                                    break
                        if too_close_to_drone:
                            continue

                        # Strict rule 3: Geofence Hard Boundary
                        proj_r = math.sqrt(proj_x * proj_x + proj_z * proj_z)
                        if proj_r > (fence_limit_r - 1.5):
                            continue

                        # Geofence boundary penalty
                        fence_penalty = 0.0
                        if proj_r > fence_warn_r:
                            excess = (proj_r - fence_warn_r) / max(1.0, fence_limit_r - fence_warn_r)
                            fence_penalty = (excess ** 2) * 120.0

                        forward_candidates.append((cand_ang, clearance, cand_idx, off_deg, fence_penalty))

                    # If no valid forward candidate, expand search to 360°
                    if not forward_candidates:
                        for off_deg in range(-180, 185, 10):
                            cand_ang = (smooth_hdg + off_deg + 360.0) % 360.0
                            cand_idx = int(cand_ang / 10.0) % 36
                            clearance = sector_clearance[cand_idx]
                            if clearance < 3.2:
                                continue
                            c_rad = math.radians(cand_ang)
                            proj_dist = 2.0 * 2.0
                            proj_x = px + proj_dist * math.sin(c_rad)
                            proj_z = pz - proj_dist * math.cos(c_rad)
                            proj_r = math.sqrt(proj_x * proj_x + proj_z * proj_z)
                            if proj_r > (fence_limit_r - 1.0):
                                continue
                            fence_penalty = 0.0
                            if proj_r > fence_warn_r:
                                excess = (proj_r - fence_warn_r) / max(1.0, fence_limit_r - fence_warn_r)
                                fence_penalty = (excess ** 2) * 120.0
                            forward_candidates.append((cand_ang, clearance, cand_idx, off_deg, fence_penalty))

                    # Ultimate Fallback: Turn back towards center of geofence
                    if not forward_candidates:
                        inward_ang = (math.degrees(math.atan2(-px, pz)) + 360) % 360
                        inward_idx = int(inward_ang / 10.0) % 36
                        forward_candidates = [(inward_ang, 4.0, inward_idx, 0, 0.0)]

                    # 9. Multi-Objective Scoring: Clearance + Goal + Hysteresis - Repulsion - Boundary Penalty
                    best_score = -999999.0
                    best_ang = smooth_hdg
                    best_clr = 40.0

                    for cand_ang, clearance, cand_idx, off_deg, fence_penalty in forward_candidates:
                        diff_goal = abs((cand_ang - target_goal_deg + 180) % 360 - 180)
                        diff_smooth = abs(off_deg)

                        hysteresis_bonus = 0.0
                        if avoid_side != 0:
                            if (avoid_side > 0 and off_deg > 10) or (avoid_side < 0 and off_deg < -10):
                                hysteresis_bonus = 25.0

                        score = (
                            (180.0 - diff_goal) * 1.5 +
                            min(clearance, 25.0) * 4.0 -
                            diff_smooth * 1.2 +
                            hysteresis_bonus -
                            drone_repulsion[cand_idx] -
                            fence_penalty
                        )

                        if score > best_score:
                            best_score = score
                            best_ang = cand_ang
                            best_clr = clearance

                    # 10. Rate-Limited Continuous Angular Smoothing (Max 4.5° per 100ms = 45°/s)
                    max_turn_rate_per_step = 4.5
                    ang_diff = (best_ang - smooth_hdg + 180) % 360 - 180
                    ang_diff_clamped = max(-max_turn_rate_per_step, min(max_turn_rate_per_step, ang_diff))
                    smooth_hdg = (smooth_hdg + ang_diff_clamped + 360.0) % 360.0
                    s["smooth_hdg"] = smooth_hdg

                    # 11. Speed Control: Smooth deceleration near walls (< 6.0m) or when turning
                    turn_severity = abs(ang_diff_clamped) / max_turn_rate_per_step
                    v_base = 2.6
                    if best_clr < 6.5:
                        speed = max(0.8, v_base * ((best_clr - MIN_OBS_MARGIN) / 2.5))
                    else:
                        speed = v_base
                    speed = max(0.7, speed * (1.0 - 0.35 * turn_severity))

                    # 12. Velocity & Lookahead Vectors (+X=East, +Z=South, 0°=North)
                    final_rad = math.radians(smooth_hdg)
                    cmd_vx = speed * math.sin(final_rad) + emergency_drone_push[0]
                    cmd_vz = -speed * math.cos(final_rad) + emergency_drone_push[1]

                    # 13. Forward Trajectory Projection
                    pred_pts = [[round(px, 2), round(target_alt, 2), round(pz, 2)]]
                    sim_x, sim_z = px, pz
                    sim_hdg = smooth_hdg
                    for step in range(15):
                        s_rad = math.radians(sim_hdg)
                        sim_x += speed * 0.3 * math.sin(s_rad)
                        sim_z += -speed * 0.3 * math.cos(s_rad)
                        pred_pts.append([round(sim_x, 2), round(target_alt, 2), round(sim_z, 2)])

                    d["predicted_trajectory"] = pred_pts
                    d["survey_waypoints"] = []
                    d["flight_phase"] = f"FRONTIER SCAN ({best_clr:.1f}m CLEAR)"
                    d["last_status_msg"] = f"Surveying | Obs: {min_obs_all:.1f}m (≥4.0m) | Swarm: {min_other_drone_dist:.1f}m (≥3.0m)"

                    # 14. Send Non-Blocking Guided Velocity & Lookahead Target
                    vn = -cmd_vz
                    ve = cmd_vx
                    vd = 0.0

                    lookahead_x = px + cmd_vx * 1.8
                    lookahead_z = pz + cmd_vz * 1.8

                    if self.origin is not None:
                        o_lat, o_lon, _ = self.origin
                        tgt_lat = o_lat + (-lookahead_z / 111320.0)
                        tgt_lon = o_lon + (lookahead_x / (111320.0 * math.cos(math.radians(o_lat))))
                        target_heading_rad = math.atan2(ve, vn) if (ve * ve + vn * vn) > 0.05 else math.radians(smooth_hdg)

                        conn.mav.set_position_target_global_int_send(
                            0, sysid, 1,
                            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                            0b0000100000000000,
                            int(tgt_lat * 1e7), int(tgt_lon * 1e7), float(target_alt),
                            float(vn), float(ve), float(vd),
                            0, 0, 0,
                            target_heading_rad, 0
                        )
            except Exception as e:
                print(f"[FREE_EXPLORATION_ERROR] {e}")

            time.sleep(0.1)

    def execute_command(self, cmd_data):
        """Execute command received from the web interface."""
        action = cmd_data.get("action")
        target = cmd_data.get("target", "all")
        params = cmd_data.get("params", {})

        if action == "restart_system":
            num_drones = int(params.get("num_drones", len(self.drones) or 1))
            print(f"[GCS] Full system restart requested for {num_drones} drone(s)...")
            base_dir = os.path.dirname(os.path.abspath(__file__))
            script_path = os.path.join(base_dir, "..", "restart_simulation.sh")
            if not os.path.exists(script_path):
                script_path = os.path.join(base_dir, "restart_simulation.sh")
            if not os.path.exists(script_path):
                script_path = "/home/dhairya/UAV-X-SWARM-STUDIO/restart_simulation.sh"
            subprocess.Popen(["/bin/bash", script_path, str(num_drones)], start_new_session=True)
            return

        if action == "update_settings":
            if "geofence_radius" in params:
                self.geofence_radius = float(params["geofence_radius"])
            if "geofence_alt_max" in params:
                self.geofence_alt_max = float(params["geofence_alt_max"])
            if "geofence_enabled" in params:
                self.geofence_enabled = bool(params["geofence_enabled"])
            if "avoid_margin" in params:
                self.avoid_margin = float(params["avoid_margin"])
            if "swarm_repel" in params:
                self.swarm_repel = float(params["swarm_repel"])
            if "min_drone_dist" in params:
                self.min_drone_dist = float(params["min_drone_dist"])
            if "solidify_rate" in params:
                self.solidify_rate = float(params["solidify_rate"])
            if "spawn_num_drones" in params:
                self.spawn_num_drones = int(params["spawn_num_drones"])

            self.save_settings()

            # Push live MAVLink parameter updates to all connected SITL simulation drones
            for d_id, d in self.drones.items():
                conn = d.get("conn")
                if conn and d.get("connected"):
                    try:
                        conn.mav.param_set_send(
                            d_id, 1,
                            b"FENCE_RADIUS\x00\x00\x00\x00",
                            float(self.geofence_radius),
                            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
                        )
                        conn.mav.param_set_send(
                            d_id, 1,
                            b"FENCE_ALT_MAX\x00\x00\x00",
                            float(self.geofence_alt_max),
                            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
                        )
                    except Exception:
                        pass
            print(f"[GCS] Persistent settings saved & synced to simulation: {params}")
            return

        if action == "set_geofence":
            self.geofence_enabled = bool(params.get("enabled", self.geofence_enabled))
            if "radius" in params:
                self.geofence_radius = float(params["radius"])
            if "alt_max" in params:
                self.geofence_alt_max = float(params["alt_max"])
            self.save_settings()
            print(f"[GCS] Geofence saved & updated: enabled={self.geofence_enabled}, radius={self.geofence_radius}m, alt_max={self.geofence_alt_max}m")
            return

        if action == "kill_drone" or (action == "fault_inject" and params.get("type") in ("failure", "crash", "disconnect")):
            target_id = int(params.get("drone_id", target if str(target).isdigit() else 1))
            with self.lock:
                if target_id in self.drones:
                    d = self.drones[target_id]
                    conn = d.get("conn")
                    px, py, pz = d["x"], d["y"], d["z"]
                    lat, lon = d["lat"], d["lon"]
                    
                    # 1. Kill & Disarm Drone immediately
                    d["failed"] = True
                    d["armed"] = False
                    d["connected"] = False
                    d["mode"] = "FAILED / CRASHED"
                    d["flight_phase"] = "💀 MOTOR CUT / CRASHED"
                    d["last_status_msg"] = f"💥 CRITICAL: UAV {target_id} crashed/disconnected at ({px:.1f}, {pz:.1f})"
                    if target_id in self.guided_survey_state:
                        del self.guided_survey_state[target_id]
                    d["predicted_trajectory"] = []

                    if conn:
                        try:
                            # Force disarm motor cutoff
                            conn.mav.command_long_send(
                                target_id, 1,
                                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                0, 0, 21196, 0, 0, 0, 0, 0
                            )
                        except Exception:
                            pass

                    # 2. Record downed drone location
                    self.downed_drones[target_id] = {
                        "id": target_id,
                        "x": px, "y": py, "z": pz,
                        "lat": lat, "lon": lon,
                        "timestamp": time.time(),
                        "assigned_rescuer": None
                    }

                    # 3. Autonomous Swarm Reconfiguration: Find Nearest Healthy Drone
                    best_rescuer = None
                    min_dist = 999999.0
                    for other_id, other_d in self.drones.items():
                        if other_id != target_id and other_d.get("connected") and other_d.get("armed") and not other_d.get("failed"):
                            dist_to_downed = math.sqrt((other_d["x"] - px)**2 + (other_d["z"] - pz)**2)
                            if dist_to_downed < min_dist:
                                min_dist = dist_to_downed
                                best_rescuer = other_id

                    if best_rescuer is not None:
                        self.downed_drones[target_id]["assigned_rescuer"] = best_rescuer
                        r_drone = self.drones[best_rescuer]
                        r_drone["flight_phase"] = f"SAR / RESCUE UAV {target_id}"
                        r_drone["last_status_msg"] = f"🚨 Swarm Reconfigured: Dispatched to inspect downed UAV {target_id} ({min_dist:.1f}m away)"
                        
                        # Re-route the rescuer drone towards downed coordinates
                        sar_ang = (math.degrees(math.atan2(px - r_drone["x"], -(pz - r_drone["z"]))) + 360) % 360
                        if best_rescuer in self.guided_survey_state:
                            s = self.guided_survey_state[best_rescuer]
                            s["primary_sector_deg"] = sar_ang
                            s["smooth_hdg"] = sar_ang
                        elif r_drone.get("conn"):
                            sar_alt = max(4.0, r_drone.get("alt", 5.0))
                            r_drone["conn"].set_mode(4) # GUIDED
                            if self.origin is not None:
                                o_lat, o_lon, _ = self.origin
                                tgt_lat = o_lat + (-pz / 111320.0)
                                tgt_lon = o_lon + (px / (111320.0 * math.cos(math.radians(o_lat))))
                                r_drone["conn"].mav.set_position_target_global_int_send(
                                    0, best_rescuer, 1,
                                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                    0b0000111111111000,
                                    int(tgt_lat * 1e7), int(tgt_lon * 1e7), float(sar_alt),
                                    0, 0, 0, 0, 0, 0, 0, 0
                                )
                        print(f"[SWARM_RECONFIG] UAV {target_id} DOWNED! Nearest UAV {best_rescuer} dispatched for SAR ({min_dist:.1f}m).")
            return

        if action == "revive_drone":
            target_id = int(params.get("drone_id", target if str(target).isdigit() else 1))
            with self.lock:
                to_revive = list(self.drones.keys()) if (target == "all" or target_id == 0) else ([target_id] if target_id in self.drones else [])
                for tid in to_revive:
                    d = self.drones[tid]
                    conn = d.get("conn")
                    d["failed"] = False
                    d["connected"] = True
                    d["mode"] = "GUIDED"
                    d["flight_phase"] = "RE-ARMING / RESUMING"
                    d["last_status_msg"] = f"🔄 UAV {tid} revived! Re-arming motors & resuming swarm mission..."
                    if tid in self.downed_drones:
                        del self.downed_drones[tid]

                    # 1. Re-arm motors in SITL & Command Takeoff
                    if conn:
                        try:
                            conn.set_mode(4)  # GUIDED
                            conn.mav.command_long_send(
                                tid, 1,
                                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                0, 1, 21196, 0, 0, 0, 0, 0
                            )
                            # Takeoff back to survey altitude
                            takeoff_alt = max(5.0, d.get("target_alt", 5.0))
                            conn.mav.command_long_send(
                                tid, 1,
                                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                0, 0, 0, 0, 0, 0, 0, float(takeoff_alt)
                            )
                        except Exception as ex:
                            print(f"[REVIVE] Warning sending arm/takeoff to UAV {tid}: {ex}")

                    # 2. Re-integrate into active survey exploration state
                    sector_deg = (tid * 72.0) % 360.0
                    self.guided_survey_state[tid] = {
                        "alt": max(5.0, d.get("target_alt", 5.0)),
                        "primary_sector_deg": sector_deg,
                        "explore_radius": self.geofence_radius * 0.75 if self.geofence_enabled else 60.0,
                        "turn_dir": 1.0 if tid % 2 == 0 else -1.0,
                        "center_x": 0.0,
                        "center_z": 0.0,
                        "smooth_hdg": sector_deg,
                        "start_time": time.time(),
                        "lane_k": tid - 1
                    }
                    print(f"[SWARM] UAV {tid} revived, re-armed, and resumed mission.")
            return

        if action in ("continue_survey", "resume_survey_after_sar"):
            downed_id = params.get("downed_id")
            with self.lock:
                cleared_rescuers = []
                for d_id, d_info in list(self.downed_drones.items()):
                    if downed_id is None or str(d_id) == str(downed_id):
                        r_id = d_info.get("assigned_rescuer")
                        if r_id is not None:
                            cleared_rescuers.append(r_id)
                        d_info["sar_resolved"] = True
                        d_info["assigned_rescuer"] = None

                active_list = sorted([dr for dr in self.drones.values() if dr.get("conn") and not dr.get("failed")], key=lambda dr: dr["id"])
                N = max(1, len(active_list))
                for idx, dr in enumerate(active_list):
                    sysid = dr["id"]
                    primary_sector = (idx * 360.0 / float(N)) % 360.0
                    turn_direction = 1.0 if (idx % 2 == 0) else -1.0
                    self.guided_survey_state[sysid] = {
                        "alt": dr.get("target_alt", dr.get("alt", 5.0)),
                        "primary_sector_deg": primary_sector,
                        "turn_dir": turn_direction,
                        "explore_radius": self.geofence_radius - 2.0 if self.geofence_enabled else 60.0,
                        "center_x": 0.0,
                        "center_z": 0.0,
                        "smooth_hdg": primary_sector,
                        "start_time": time.time(),
                        "lane_k": idx
                    }
                    dr["sar_arrived"] = False
                    dr["sar_downed_id"] = None
                    dr["flight_phase"] = f"RESUMED SURVEY ({primary_sector:.0f}° SECTOR)"
                    dr["last_status_msg"] = f"Resumed autonomous survey: Sector {primary_sector:.0f}°"
                    if dr.get("conn"):
                        dr["conn"].set_mode(4)
                self.guided_survey_active = True
                print(f"[SWARM] Swarm autonomous survey resumed after SAR inspection. Fleet: {[dr['id'] for dr in active_list]}")
            return

        if action == "recenter_swarm":
            with self.lock:
                for sysid, s in self.guided_survey_state.items():
                    if sysid in self.drones:
                        dr = self.drones[sysid]
                        px, pz = dr["x"], dr["z"]
                        return_ang = (math.degrees(math.atan2(-px, pz)) + 360) % 360
                        s["primary_sector_deg"] = return_ang
                        s["smooth_hdg"] = return_ang
                        dr["flight_phase"] = "RECENTERING SWARM"
                        dr["last_status_msg"] = "Recentering trajectory towards home coordinates."
                print("[SWARM] Swarm commanded to recenter towards origin.")
            return

        target_drones = []
        with self.lock:
            if target == "all":
                target_drones = [d for d in self.drones.values() if d["conn"]]
            elif isinstance(target, int) and target in self.drones:
                if self.drones[target]["conn"]:
                    target_drones = [self.drones[target]]
            elif isinstance(target, str) and target.isdigit() and int(target) in self.drones:
                if self.drones[int(target)]["conn"]:
                    target_drones = [self.drones[int(target)]]

        def _dispatch_single_drone(d):
            conn = d["conn"]
            sysid = d["id"]
            try:
                if action == "arm":
                    d["last_status_msg"] = "Arming motors commanded..."
                    conn.set_mode(4) # GUIDED
                    conn.mav.command_long_send(sysid, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
                elif action == "disarm":
                    d["last_status_msg"] = "Disarming motors commanded"
                    if sysid in self.guided_survey_state:
                        del self.guided_survey_state[sysid]
                    d["predicted_trajectory"] = []
                    conn.mav.command_long_send(sysid, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 21196, 0, 0, 0, 0, 0)
                elif action == "takeoff":
                    alt = float(params.get("alt", 3.0))
                    d["target_alt"] = alt
                    d["last_status_msg"] = f"Takeoff commanded to {alt:.1f}m"
                    conn.set_mode(4) # GUIDED
                    conn.mav.command_long_send(sysid, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
                    time.sleep(0.1)
                    conn.mav.command_long_send(sysid, 1, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt)
                elif action == "alt":
                    target_alt = float(params.get("alt", 5.0))
                    d["target_alt"] = target_alt
                    conn.set_mode(4) # GUIDED
                    if d["alt"] < 0.6:
                        d["last_status_msg"] = f"Takeoff to altitude: {target_alt:.1f}m"
                        conn.mav.command_long_send(sysid, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
                        time.sleep(0.1)
                        conn.mav.command_long_send(sysid, 1, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, target_alt)
                    else:
                        d["last_status_msg"] = f"Climbing to target altitude: {target_alt:.1f}m"
                        conn.mav.set_position_target_global_int_send(
                            0, sysid, 1,
                            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                            0b0000110111111011,
                            0, 0,
                            float(target_alt),
                            0, 0, 0, 0, 0, 0, 0, 0
                        )
                elif action == "mode":
                    mode_name = str(params.get("mode", "GUIDED")).upper()
                    mode_map = conn.mode_mapping() if hasattr(conn, 'mode_mapping') and conn.mode_mapping() else MODE_STR_TO_NUM
                    mode_id = mode_map.get(mode_name, 4)
                    d["last_status_msg"] = f"Flight mode set to {mode_name}"
                    if sysid in self.guided_survey_state:
                        del self.guided_survey_state[sysid]
                    d["predicted_trajectory"] = []
                    conn.set_mode(mode_id)
                elif action == "survey":
                    length_m = float(params.get("length", 50.0))
                    width_m = float(params.get("width", 40.0))
                    base_alt = float(params.get("height", 6.0))
                    heading_deg = float(params.get("heading", 0.0))
                    altitudes_dict = params.get("altitudes", {})
                    is_guided = bool(params.get("guided", True))
                    auto_start = bool(params.get("auto_start", True))

                    active_list = sorted([dr for dr in self.drones.values() if dr["conn"] and dr["lat"] != 0.0], key=lambda dr: dr["id"])
                    if not active_list:
                        active_list = sorted([dr for dr in self.drones.values() if dr["conn"]], key=lambda dr: dr["id"])

                    N = max(1, len(active_list))
                    k = 0
                    for idx, dr in enumerate(active_list):
                        if dr["id"] == sysid:
                            k = idx
                            break

                    drone_alt = float(altitudes_dict.get(str(sysid), altitudes_dict.get(sysid, base_alt)))
                    drone_alt = max(1.5, min(drone_alt, 30.0))
                    d["target_alt"] = drone_alt

                    ref_x = sum(dr["x"] for dr in active_list) / float(N) if active_list else d["x"]
                    ref_z = sum(dr["z"] for dr in active_list) / float(N) if active_list else d["z"]

                    # Distributed Multi-Drone Exploration Sectors (Fanning out 360° across the unknown zone)
                    primary_sector = (heading_deg + (k * 360.0 / float(N))) % 360.0
                    turn_direction = 1.0 if (k % 2 == 0) else -1.0
                    explore_rad = max(45.0, max(length_m, width_m) * 0.8)

                    if is_guided:
                        self.guided_survey_state[sysid] = {
                            "alt": drone_alt,
                            "primary_sector_deg": primary_sector,
                            "turn_dir": turn_direction,
                            "explore_radius": explore_rad,
                            "center_x": ref_x,
                            "center_z": ref_z,
                            "smooth_hdg": primary_sector,
                            "start_time": time.time(),
                            "lane_k": k
                        }
                        self.guided_survey_active = True
                        d["survey_waypoints"] = []
                        if not d.get("armed", False) and (not d.get("ekf_ready", False) or d.get("alt", 0.0) < 0.5):
                            d["last_status_msg"] = "⏳ Survey Queued: Waiting for EKF lock..."
                            d["flight_phase"] = "QUEUED (WAITING EKF)"
                        else:
                            d["last_status_msg"] = f"Frontier Exploration: Sector {primary_sector:.0f}° @ {drone_alt:.1f}m"
                            d["flight_phase"] = f"EXPLORING SECTOR {primary_sector:.0f}°"
                        conn.set_mode(4)  # GUIDED
                    else:
                        gps_drones = [dr for dr in active_list if dr["lat"] != 0.0 and dr["lon"] != 0.0]
                        if gps_drones:
                            ref_lat = sum(dr["lat"] for dr in gps_drones) / len(gps_drones)
                            ref_lon = sum(dr["lon"] for dr in gps_drones) / len(gps_drones)
                        elif self.origin is not None:
                            ref_lat, ref_lon, _ = self.origin
                        else:
                            ref_lat, ref_lon = d["lat"], d["lon"]

                        home_lat = d["lat"] if d["lat"] != 0.0 else ref_lat
                        home_lon = d["lon"] if d["lon"] != 0.0 else ref_lon

                        def to_gps(x_loc, y_loc):
                            dn = y_loc * math.cos(psi) - x_loc * math.sin(psi)
                            de = y_loc * math.sin(psi) + x_loc * math.cos(psi)
                            lat = ref_lat + (dn / 111320.0)
                            lon = ref_lon + (de / (111320.0 * math.cos(math.radians(ref_lat))))
                            return lat, lon

                        p1_lat, p1_lon = to_gps(sub_l, 0.0)
                        p2_lat, p2_lon = to_gps(sub_l, length_m)
                        p3_lat, p3_lon = to_gps(sub_r, length_m)
                        p4_lat, p4_lon = to_gps(sub_r, 0.0)

                        wps = [
                            {'lat': home_lat, 'lon': home_lon, 'alt': drone_alt, 'command': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT},
                            {'lat': p1_lat,   'lon': p1_lon,   'alt': drone_alt, 'command': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT},
                            {'lat': p2_lat,   'lon': p2_lon,   'alt': drone_alt, 'command': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT},
                            {'lat': p3_lat,   'lon': p3_lon,   'alt': drone_alt, 'command': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT},
                            {'lat': p4_lat,   'lon': p4_lon,   'alt': drone_alt, 'command': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT},
                            {'lat': home_lat, 'lon': home_lon, 'alt': drone_alt, 'command': mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH},
                        ]

                        self.pending_missions[sysid] = wps
                        self.pending_auto_start[sysid] = auto_start
                        d["last_status_msg"] = f"Uploading AUTO Mission: {length_m:.0f}x{width_m:.0f}m @ {drone_alt:.1f}m..."
                        d["flight_phase"] = f"SURVEY UPLOAD ({drone_alt:.1f}m)"
                        conn.mav.mission_clear_all_send(sysid, 1)
                        time.sleep(0.05)
                        conn.mav.mission_count_send(sysid, 1, len(wps), 0)
                elif action == "auto":
                    d["last_status_msg"] = "AUTO survey mission started"
                    conn.mav.mission_set_current_send(sysid, 1, 1)
                    conn.mav.command_long_send(sysid, 1, mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT, 0, 1, 0, 0, 0, 0, 0, 0)
                    conn.mav.command_long_send(sysid, 1, mavutil.mavlink.MAV_CMD_MISSION_START, 0, 1, 0, 0, 0, 0, 0, 0)
                    conn.set_mode(3) # AUTO
                elif action == "rtl":
                    d["last_status_msg"] = "RTL (Return To Launch) commanded"
                    if sysid in self.guided_survey_state:
                        del self.guided_survey_state[sysid]
                    d["predicted_trajectory"] = []
                    conn.set_mode(6) # RTL
                elif action == "land":
                    d["last_status_msg"] = "Landing commanded"
                    if sysid in self.guided_survey_state:
                        del self.guided_survey_state[sysid]
                    d["predicted_trajectory"] = []
                    conn.set_mode(9) # LAND
                elif action == "brake":
                    d["last_status_msg"] = "Brake commanded (Position hold)"
                    if sysid in self.guided_survey_state:
                        del self.guided_survey_state[sysid]
                    d["predicted_trajectory"] = []
                    conn.set_mode(17) # BRAKE
                elif action == "heading":
                    yaw = float(params.get("yaw", 0.0))
                    d["last_status_msg"] = f"Heading locked to {yaw:.0f}°"
                    conn.set_mode(4)
                    conn.mav.command_long_send(sysid, 1, mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0, yaw, 60.0, 1, 0, 0, 0, 0)
                elif action == "yaw_relative":
                    delta_yaw = float(params.get("delta_yaw", 0.0))
                    cur_hdg = d["heading"]
                    new_yaw = (cur_hdg + delta_yaw) % 360
                    d["last_status_msg"] = f"Yaw rotated {delta_yaw:+.0f}° -> {new_yaw:.0f}°"
                    conn.set_mode(4) # GUIDED
                    conn.mav.command_long_send(
                        sysid, 1,
                        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                        0,
                        float(new_yaw),
                        45.0,
                        1 if delta_yaw >= 0 else -1,
                        0, # Absolute target yaw
                        0, 0, 0
                    )
                elif action == "move":
                    north = float(params.get("north", 0.0))
                    east = float(params.get("east", 0.0))
                    down = float(params.get("down", 0.0))
                    climb = float(params.get("climb", 0.0))
                    delta_alt = climb - down
                    yaw_deg = params.get("yaw")
                    conn.set_mode(4) # GUIDED
                    
                    if not d["armed"] or d["alt"] < 0.6:
                        takeoff_alt = max(3.0, 3.0 + delta_alt)
                        d["target_alt"] = takeoff_alt
                        d["last_status_msg"] = f"Auto-takeoff to {takeoff_alt:.1f}m before moving"
                        conn.mav.command_long_send(sysid, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
                        time.sleep(0.1)
                        conn.mav.command_long_send(sysid, 1, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, takeoff_alt)
                    elif d["lat"] != 0.0 and d["lon"] != 0.0:
                        new_lat = d["lat"] + (north / 111320.0)
                        new_lon = d["lon"] + (east / (111320.0 * math.cos(math.radians(d["lat"]))))
                        new_alt = max(1.0, d["alt"] + delta_alt)
                        d["target_alt"] = new_alt
                        d["last_status_msg"] = f"Moving N:{north:+.1f}m E:{east:+.1f}m Alt:{new_alt:.1f}m"
                        typemask = 0b0000100111111000 if yaw_deg is not None else 0b0000111111111000
                        yaw_rad = math.radians(float(yaw_deg) % 360) if yaw_deg is not None else 0.0
                        conn.mav.set_position_target_global_int_send(
                            0, sysid, 1,
                            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                            typemask,
                            int(new_lat * 1e7), int(new_lon * 1e7), float(new_alt),
                            0, 0, 0, 0, 0, 0,
                            yaw_rad, 0
                        )
                elif action == "velocity":
                    vx = float(params.get("vx", 0.0))
                    vy = float(params.get("vy", 0.0))
                    vz = float(params.get("vz", 0.0))
                    yaw_rate = float(params.get("yaw_rate", 0.0))
                    conn.set_mode(4) # GUIDED
                    conn.mav.set_position_target_local_ned_send(
                        0, sysid, 1,
                        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                        0b0000101111000111,
                        0, 0, 0,
                        vx, vy, vz,
                        0, 0, 0,
                        0, yaw_rate
                    )
                elif action == "brake":
                    d["last_status_msg"] = "Brake commanded (Position hold)"
                    conn.set_mode(17) # BRAKE
                elif action == "formation":
                    form_type = params.get("type", "line").lower()
                    spacing = float(params.get("spacing", 4.0))
                    radius = float(params.get("radius", 8.0))
                    heading_deg = float(params.get("heading", 0.0))
                    base_alt = float(params.get("alt", 0.0))

                    gps_drones = [dr for dr in self.drones.values() if dr["conn"] and dr["lat"] != 0.0 and dr["lon"] != 0.0]
                    active_list = sorted(gps_drones, key=lambda dr: dr["id"])
                    if not active_list:
                        active_list = sorted([dr for dr in self.drones.values() if dr["conn"]], key=lambda dr: dr["id"])

                    N = max(1, len(active_list))

                    # Compute live swarm centroid reference from active drones
                    if gps_drones:
                        c_lat = sum(dr["lat"] for dr in gps_drones) / len(gps_drones)
                        c_lon = sum(dr["lon"] for dr in gps_drones) / len(gps_drones)
                        c_alt = sum(dr["alt"] for dr in gps_drones) / len(gps_drones)
                    elif self.origin is not None:
                        c_lat, c_lon, _ = self.origin
                        c_alt = 5.0
                    else:
                        c_lat, c_lon, c_alt = 0.0, 0.0, 5.0

                    if base_alt < 1.0:
                        base_alt = max(3.0, c_alt)

                    theta = math.radians(heading_deg)

                    def _dispatch_formation_drone(dr, k):
                        sysid = dr["id"]
                        conn = dr["conn"]
                        if not conn:
                            return

                        dx, dy, dz_offset = 0.0, 0.0, 0.0

                        if form_type == "line":
                            s = (k - (N - 1) / 2.0) * spacing
                            dx = s * math.cos(theta)
                            dy = s * math.sin(theta)
                        elif form_type == "v_shape":
                            if k == 0:
                                dx, dy = 0.0, 0.0
                            else:
                                side = 1.0 if (k % 2 == 1) else -1.0
                                tier = (k + 1) // 2
                                beta = math.radians(35.0)
                                x_local = side * tier * spacing * math.sin(beta)
                                y_local = -tier * spacing * math.cos(beta)
                                dx = x_local * math.cos(theta) - y_local * math.sin(theta)
                                dy = x_local * math.sin(theta) + y_local * math.cos(theta)
                        elif form_type == "grid":
                            cols = max(2, int(math.ceil(math.sqrt(N))))
                            row = k // cols
                            col = k % cols
                            rows_tot = int(math.ceil(N / cols))
                            x_local = (col - (cols - 1) / 2.0) * spacing
                            y_local = (row - (rows_tot - 1) / 2.0) * spacing
                            dx = x_local * math.cos(theta) - y_local * math.sin(theta)
                            dy = x_local * math.sin(theta) + y_local * math.cos(theta)
                        elif form_type == "circle":
                            phi = theta + (2.0 * math.pi * k / N)
                            dx = radius * math.cos(phi)
                            dy = radius * math.sin(phi)
                        elif form_type == "helix":
                            phi = theta + (2.0 * math.pi * k / N)
                            dx = (radius + k * 1.5) * math.cos(phi)
                            dy = (radius + k * 1.5) * math.sin(phi)
                            dz_offset = (k - (N - 1) / 2.0) * 1.5

                        final_alt = max(2.0, base_alt + dz_offset)
                        dr["target_alt"] = final_alt

                        ref_lat = c_lat if c_lat != 0.0 else dr["lat"]
                        ref_lon = c_lon if c_lon != 0.0 else dr["lon"]

                        if ref_lat == 0.0 and ref_lon == 0.0:
                            print(f"[GCS] Cannot execute formation on D{sysid}: No GPS fix")
                            return

                        d_lat = dy / 111320.0
                        d_lon = dx / (111320.0 * math.cos(math.radians(ref_lat)))
                        tgt_lat = ref_lat + d_lat
                        tgt_lon = ref_lon + d_lon

                        dr["last_status_msg"] = f"Formation {form_type.upper()}: Slot {k+1} -> [{dx:+.1f}m, {dy:+.1f}m, {final_alt:.1f}m]"
                        dr["flight_phase"] = f"CHOREOGRAPHY ({form_type.upper()})"

                        conn.set_mode(4) # GUIDED
                        
                        if not dr["armed"] or dr["alt"] < 0.8:
                            conn.mav.command_long_send(sysid, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
                            time.sleep(0.3)
                            conn.mav.command_long_send(sysid, 1, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, float(final_alt))
                            
                            def _send_waypoint_after_liftoff():
                                for _ in range(35):
                                    time.sleep(0.2)
                                    if dr["alt"] >= 1.2:
                                        break
                                time.sleep(0.5)
                                conn.mav.set_position_target_global_int_send(
                                    0, sysid, 1,
                                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                    0b0000100111111000,
                                    int(tgt_lat * 1e7), int(tgt_lon * 1e7), float(final_alt),
                                    0, 0, 0, 0, 0, 0,
                                    theta, 0
                                )
                            threading.Thread(target=_send_waypoint_after_liftoff, daemon=True).start()
                        else:
                            conn.mav.set_position_target_global_int_send(
                                0, sysid, 1,
                                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                0b0000100111111000,
                                int(tgt_lat * 1e7), int(tgt_lon * 1e7), float(final_alt),
                                0, 0, 0, 0, 0, 0,
                                theta, 0
                            )

                    for idx, dr in enumerate(target_drones):
                        k = idx
                        for a_idx, a_dr in enumerate(active_list):
                            if a_dr["id"] == dr["id"]:
                                k = a_idx
                                break
                        t = threading.Thread(target=_dispatch_formation_drone, args=(dr, k), daemon=True)
                        t.start()
            except Exception as e:
                print(f"[GCS] Error dispatching {action} to D{sysid}: {e}")

        # Launch dispatch in concurrent threads for instant execution across all drones
        for d in target_drones:
            t = threading.Thread(target=_dispatch_single_drone, args=(d,), daemon=True)
            t.start()


manager = SwarmTelemetryManager()


async def ws_handler(websocket):
    manager.connected_ws.add(websocket)
    try:
        async for message in websocket:
            try:
                cmd = json.loads(message)
                manager.execute_command(cmd)
            except Exception as e:
                print(f"[WS] Command parse error: {e}")
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        manager.connected_ws.discard(websocket)


async def telemetry_broadcaster():
    """Streams live swarm telemetry at 25Hz to all connected browser clients."""
    while True:
        if manager.connected_ws:
            payload = manager.get_snapshot()
            data_str = json.dumps(payload)
            websockets_list = list(manager.connected_ws)
            if websockets_list:
                await asyncio.gather(*[ws.send(data_str) for ws in websockets_list], return_exceptions=True)
        await asyncio.sleep(0.04)


class NoCacheHTTPRequestHandler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
        # Handle Live Camera Snapshot & Video Streaming
        if self.path.startswith("/api/camera"):
            parsed = urllib.parse.urlparse(self.path)
            query = urllib.parse.parse_qs(parsed.query)
            drone_id = int(query.get("drone", query.get("id", ["1"]))[0])
            cam_name = query.get("camera", query.get("cam", ["0"]))[0]
            img_type = int(query.get("type", ["0"])[0])

            if "stream" in parsed.path:
                self.send_response(200)
                self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=--frame')
                self.send_header('Cache-Control', 'no-cache, private')
                self.end_headers()
                try:
                    while True:
                        frame_bytes = manager.fetch_camera_frame(drone_id, cam_name, img_type)
                        if frame_bytes:
                            self.wfile.write(b"--frame\r\n")
                            self.wfile.write(b"Content-Type: image/png\r\n\r\n")
                            self.wfile.write(frame_bytes)
                            self.wfile.write(b"\r\n")
                        time.sleep(0.08) # ~12 FPS
                except Exception:
                    return
            else:
                frame_bytes = manager.fetch_camera_frame(drone_id, cam_name, img_type)
                if frame_bytes:
                    self.send_response(200)
                    self.send_header('Content-type', 'image/png')
                    self.send_header('Content-length', str(len(frame_bytes)))
                    self.end_headers()
                    self.wfile.write(frame_bytes)
                    return
                else:
                    self.send_response(204)
                    self.end_headers()
                    return

        super().do_GET()

    def end_headers(self):
        self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
        super().end_headers()

class ThreadingHTTPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    daemon_threads = True
    allow_reuse_address = True

def run_http_server():
    os.chdir(STATIC_DIR)
    handler = NoCacheHTTPRequestHandler
    with ThreadingHTTPServer(("", HTTP_PORT), handler) as httpd:
        print(f"\033[1;32m★ UAV-X Swarm Studio Web GCS is running at: http://localhost:{HTTP_PORT}\033[0m")
        httpd.serve_forever()


async def main():
    manager.init_drones()
    
    http_thread = threading.Thread(target=run_http_server, daemon=True)
    http_thread.start()

    print(f"\033[1;36m★ WebSocket Telemetry Server running on ws://localhost:{WS_PORT}\033[0m")
    
    async with websockets.serve(ws_handler, "0.0.0.0", WS_PORT):
        await telemetry_broadcaster()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nShutting down Swarm Studio.")
