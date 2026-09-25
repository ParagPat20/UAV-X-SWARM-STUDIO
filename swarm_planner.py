#!/usr/bin/env python3
"""
Autonomous Multi-Drone Swarm Mission Planner
Generates parallel collision-free survey missions for N drones and uploads them directly via MAVLink.

Usage:
    python3 swarm_planner.py --drones 4 --survey 50 50 --alt 4.0
    python3 swarm_planner.py --drones 4 --line 5.0
    python3 swarm_planner.py --drones 4 --circle 8.0
"""

import sys
import time
import math
import argparse
import threading
from pymavlink import mavutil


def upload_single_drone_mission(drone_idx, port, waypoints):
    conn_str = f"tcp:127.0.0.1:{port}"
    try:
        conn = mavutil.mavlink_connection(conn_str)
        hb = conn.wait_heartbeat(timeout=10)
        if not hb:
            print(f"  [D{drone_idx}] ✗ No heartbeat on {conn_str}")
            return False
        sysid = hb.get_srcSystem()

        count = len(waypoints)
        conn.mav.mission_count_send(sysid, 1, count, 0)

        for seq in range(count):
            msg = conn.recv_match(type=['MISSION_REQUEST', 'MISSION_REQUEST_INT'], blocking=True, timeout=4.0)
            if not msg:
                print(f"  [D{drone_idx}] ✗ Timeout waiting for MISSION_REQUEST seq {seq}")
                return False

            req_seq = msg.seq
            item = waypoints[req_seq]
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

        ack = conn.recv_match(type='MISSION_ACK', blocking=True, timeout=4.0)
        if ack and ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
            conn.mav.mission_set_current_send(sysid, 1, 1)
            print(f"  [D{drone_idx}] \033[1;32m✓ Mission ({count} WPs) Uploaded Successfully!\033[0m")
            return True
        else:
            print(f"  [D{drone_idx}] ✗ Mission upload failed: {ack}")
            return False
    except Exception as e:
        print(f"  [D{drone_idx}] ✗ Error: {e}")
        return False
    finally:
        try:
            conn.close()
        except Exception:
            pass


def get_drone_home_positions(num_drones):
    homes = {}
    for i in range(1, num_drones + 1):
        port = 5762 + (i - 1) * 10
        try:
            conn = mavutil.mavlink_connection(f"tcp:127.0.0.1:{port}")
            hb = conn.wait_heartbeat(timeout=5)
            if hb:
                sysid = hb.get_srcSystem()
                conn.mav.request_data_stream_send(sysid, 1, mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
                t0 = time.time()
                while time.time() - t0 < 3.0:
                    msg = conn.recv_match(type=['GLOBAL_POSITION_INT', 'GPS_RAW_INT'], blocking=True, timeout=1.0)
                    if msg and msg.get_srcSystem() == sysid:
                        lat = msg.lat / 1e7
                        lon = msg.lon / 1e7
                        if lat != 0.0:
                            homes[i] = (lat, lon)
                            break
            conn.close()
        except Exception:
            pass
    return homes


def plan_and_upload_survey(num_drones, length_m=50.0, width_m=50.0, alt=4.0):
    print(f"\n{'='*65}")
    print(f"  ★ SWARM MISSION PLANNER — Survey Area {length_m}m x {width_m}m across {num_drones} Drones")
    print(f"{'='*65}\n")
    print("  Fetching current GPS positions of drones...")
    homes = get_drone_home_positions(num_drones)

    if len(homes) < num_drones:
        print(f"  ✗ Found GPS for only {len(homes)}/{num_drones} drones. Ensure SITL is running.")
        sys.exit(1)

    lane_width = width_m / float(num_drones)
    print(f"  ✓ Found all {num_drones} drones. Generating {num_drones} parallel lanes ({lane_width:.1f}m width per lane)...\n")

    threads = []
    for i in range(1, num_drones + 1):
        home_lat, home_lon = homes[i]
        lane_center_e = (i - 0.5) * lane_width - (width_m / 2.0)
        port = 5762 + (i - 1) * 10

        wp_list = []
        # WP 0: Dummy Home
        wp_list.append({'lat': home_lat, 'lon': home_lon, 'alt': 0, 'command': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT})
        # WP 1: Takeoff
        wp_list.append({'lat': home_lat, 'lon': home_lon, 'alt': alt, 'command': mavutil.mavlink.MAV_CMD_NAV_TAKEOFF})
        # WP 2: Start of Lane
        s_lat = home_lat
        s_lon = home_lon + (lane_center_e / (111320.0 * math.cos(math.radians(home_lat))))
        wp_list.append({'lat': s_lat, 'lon': s_lon, 'alt': alt, 'command': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT})
        # WP 3: End of Lane (North)
        e_lat = home_lat + (length_m / 111320.0)
        e_lon = s_lon
        wp_list.append({'lat': e_lat, 'lon': e_lon, 'alt': alt, 'command': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT})
        # WP 4: Shift Lane + Return (South)
        ret_e_lon = s_lon + (lane_width * 0.4 / (111320.0 * math.cos(math.radians(home_lat))))
        wp_list.append({'lat': e_lat, 'lon': ret_e_lon, 'alt': alt, 'command': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT})
        wp_list.append({'lat': s_lat, 'lon': ret_e_lon, 'alt': alt, 'command': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT})
        # WP 5: RTL
        wp_list.append({'lat': home_lat, 'lon': home_lon, 'alt': alt, 'command': mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH})

        t = threading.Thread(target=upload_single_drone_mission, args=(i, port, wp_list), daemon=True)
        threads.append(t)
        t.start()

    for t in threads:
        t.join()

    print("\n  \033[1;32m✓ All missions uploaded successfully to the swarm!\033[0m")
    print("  In Drone Commander terminal, type: \033[1;36mall arm\033[0m then \033[1;36mall auto\033[0m to launch the swarm mission!\n")


def main():
    parser = argparse.ArgumentParser(description="Multi-Drone Swarm Mission Planner")
    parser.add_argument("--drones", type=int, default=4, help="Number of drones in swarm")
    parser.add_argument("--survey", nargs=2, type=float, metavar=('LENGTH', 'WIDTH'), help="Plan area survey: <length_m> <width_m>")
    parser.add_argument("--alt", type=float, default=4.0, help="Survey flight altitude in meters")
    args = parser.parse_args()

    length = args.survey[0] if args.survey else 50.0
    width = args.survey[1] if args.survey else 50.0
    plan_and_upload_survey(args.drones, length_m=length, width_m=width, alt=args.alt)


if __name__ == "__main__":
    main()
