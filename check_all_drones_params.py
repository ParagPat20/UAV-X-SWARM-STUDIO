#!/usr/bin/env python3
"""
Diagnostic tool to verify live parameters across ALL active drones in the swarm
against the baseline parameters in swarm_params.parm.
"""

import os
import sys
import time
import math
from pymavlink import mavutil

SWARM_PARM_PATH = "/home/parag/swarm_params.parm"

KEY_PARAMS_TO_VERIFY = [
    "OA_TYPE",
    "OA_BR_TYPE",
    "OA_BR_LOOKAHEAD",
    "OA_BR_CONT_ANGLE",
    "OA_BR_CONT_RATIO",
    "OA_MARGIN_MAX",
    "OA_DB_DIST_MAX",
    "OA_DB_SIZE",
    "PRX1_TYPE",
    "PRX1_MIN",
    "PRX1_MAX",
    "PRX_IGN_GND",
    "PRX_ALT_MIN",
    "AVOID_ENABLE",
    "AVOID_DIST_MAX",
    "AVOID_MARGIN",
    "AVOID_BEHAVE",
    "EK3_MAG_CAL",
    "EK3_GPS_CHECK",
    "EK3_HGT_DELAY",
    "GPS1_DELAY_MS",
    "WP_SPD",
    "MOT_THST_HOVER",
]

def load_file_params(path: str) -> dict:
    params = {}
    if not os.path.exists(path):
        return params
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) >= 2:
                try:
                    params[parts[0]] = float(parts[1])
                except ValueError:
                    pass
    return params

def check_drone_params(drone_idx: int, tcp_port: int, expected_params: dict):
    print(f"\n[Connecting] UAV {drone_idx} on TCP port {tcp_port}...")
    try:
        conn = mavutil.mavlink_connection(f"tcp:127.0.0.1:{tcp_port}")
        hb = conn.wait_heartbeat(timeout=3.0)
        if not hb:
            print(f"  ❌ UAV {drone_idx}: No heartbeat on port {tcp_port} (Drone not running or offline)")
            return None
        
        sysid = conn.target_system
        print(f"  ✓ Connected to UAV {drone_idx} (SysID {sysid}). Fetching live parameters...")

        live_params = {}
        conn.param_fetch_all()
        t_start = time.time()
        while time.time() - t_start < 6.0:
            msg = conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=1.0)
            if not msg:
                break
            live_params[msg.param_id] = float(msg.param_value)

        conn.close()

        if len(live_params) < 20:
            print(f"  ⚠️ UAV {drone_idx}: Only received {len(live_params)} params.")
            return None

        # Verify key parameters
        results = []
        all_matched = True
        for p in KEY_PARAMS_TO_VERIFY:
            exp_val = expected_params.get(p)
            live_val = live_params.get(p)
            if exp_val is None or live_val is None:
                match = (exp_val == live_val)
            else:
                match = math.isclose(exp_val, live_val, rel_tol=1e-4, abs_tol=1e-4)
            if not match:
                all_matched = False
            results.append({
                "param": p,
                "expected": exp_val,
                "live": live_val,
                "match": match
            })

        return results, len(live_params), all_matched

    except Exception as e:
        print(f"  ❌ UAV {drone_idx} Error: {e}")
        return None

def main():
    print("=" * 80)
    print("      UAV-X SWARM STUDIO — DRONE PARAMETER INTEGRITY VERIFIER")
    print("=" * 80)
    
    file_params = load_file_params(SWARM_PARM_PATH)
    print(f"Loaded {len(file_params)} baseline parameters from: {SWARM_PARM_PATH}\n")

    num_drones = 1
    if len(sys.argv) > 1:
        try:
            num_drones = max(1, int(sys.argv[1]))
        except ValueError:
            num_drones = 1

    summary = {}

    for i in range(1, num_drones + 1):
        port = 5762 + (i - 1) * 10
        res = check_drone_params(i, port, file_params)
        if res:
            results, total_params, all_matched = res
            summary[i] = (results, total_params, all_matched)

    if not summary:
        print("\n❌ No active drones detected. Please start SITL first using ./start_sitl.sh\n")
        sys.exit(1)

    print("\n" + "=" * 80)
    print(f"{'DRONE':<8} {'PARAMETER':<20} {'LIVE VALUE':<16} {'PARM FILE VALUE':<18} {'STATUS':<10}")
    print("=" * 80)

    for drone_idx, (results, total_params, all_matched) in summary.items():
        for r in results:
            exp_str = f"{r['expected']:.2f}" if r['expected'] is not None else "N/A"
            live_str = f"{r['live']:.2f}" if r['live'] is not None else "N/A"
            status_str = "✓ MATCH" if r['match'] else "❌ MISMATCH"
            print(f"UAV {drone_idx:<4} {r['param']:<20} {live_str:<16} {exp_str:<18} {status_str:<10}")
        print("-" * 80)

    print("\n" + "=" * 80)
    print("                        VERIFICATION SUMMARY")
    print("=" * 80)
    for drone_idx, (results, total_params, all_matched) in summary.items():
        if all_matched:
            print(f"★ UAV {drone_idx}: 100% MATCH — All obstacle avoidance & physics parameters active!")
        else:
            mismatches = [r['param'] for r in results if not r['match']]
            print(f"⚠️ UAV {drone_idx}: Mismatches detected on {len(mismatches)} params: {', '.join(mismatches)}")
    print("=" * 80 + "\n")

if __name__ == "__main__":
    main()
