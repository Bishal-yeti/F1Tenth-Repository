import numpy as np
import gym, f110_gym, yaml, sys, os, csv
sys.path.insert(0, os.path.abspath("src/controllers"))
from controllers.lidar_race_core import RealLidarRaceCore, PACE_TRIMS

with open("src/f1tenth_gym_ros/config/sim.yaml") as f:
    conf = yaml.safe_load(f)["bridge"]["ros__parameters"]

map_path = os.path.expanduser(conf["map_path"])
if not os.path.exists(map_path + ".yaml"):
    map_path = os.path.abspath(os.path.join("src/f1tenth_gym_ros/maps", os.path.basename(map_path.rstrip("/"))))
map_ext = conf["map_img_ext"]
sx, sy, stheta = float(conf["sx"]), float(conf["sy"]), float(conf["stheta"])
print(f"Map: {map_path}, Start: ({sx}, {sy}, {stheta})")

env = gym.make("f110_gym:f110-v0", map=map_path, map_ext=map_ext, num_agents=1)
obs, _, _, _ = env.reset(np.array([[sx, sy, stheta]]))

core = RealLidarRaceCore(adaptive_pace=True)

dist = 0.0
prev_pos = np.array([sx, sy])
time_elapsed = 0.0

laps = []
lap_num = 1
lap_start_time = 0.0
lap_start_dist = 0.0
min_clear = 999.0
max_speed = 0.0
collisions = 0

for step in range(50000): # ~500s (up to 12 laps)
    scans = obs["scans"][0]
    px, py, pth = obs["poses_x"][0], obs["poses_y"][0], obs["poses_theta"][0]
    vx = obs["linear_vels_x"][0]
    cur_pos = np.array([px, py])
    dist += float(np.linalg.norm(cur_pos - prev_pos))
    prev_pos = cur_pos
    time_elapsed += 0.01

    cur_lap_time = time_elapsed - lap_start_time
    cur_lap_dist = dist - lap_start_dist
    d_to_start = float(np.hypot(px - sx, py - sy))
    min_scan = float(np.min(scans))
    min_clear = min(min_clear, min_scan)
    max_speed = max(max_speed, vx)

    core.check_mid_lap_safety(min_scan)
    steer, speed = core.step(scans, -2.35, 4.7/1080.0, vx)
    obs, _, done, info = env.step(np.array([[steer, speed]]))

    if env.collisions[0] == 1.0:
        collisions += 1
        print(f"COLLISION at step {step} (t={time_elapsed:.2f}s, dist={dist:.1f}m, pos=({px:.2f}, {py:.2f}), min_clear={min_clear:.3f}m)")
        break

    yaw_diff = abs(np.arctan2(np.sin(pth - stheta), np.cos(pth - stheta)))
    if cur_lap_dist > 400.0 and d_to_start < 2.5 and yaw_diff < 1.0:
        used_trim = core.trim_label
        gain_vs_lap1 = (laps[0][1] - cur_lap_time) if laps else 0.0
        gain_vs_prev = (laps[-1][1] - cur_lap_time) if laps else 0.0
        laps.append((lap_num, cur_lap_time, cur_lap_dist, max_speed, min_clear, used_trim, gain_vs_lap1, gain_vs_prev))
        print(f"Lap {lap_num:2d} [{used_trim}] finished: {cur_lap_time:6.3f}s | Gain vs Lap1: {gain_vs_lap1:+6.3f}s | Delta Prev: {gain_vs_prev:+6.3f}s | Peak: {max_speed:5.2f}m/s | MinClear: {min_clear:5.3f}m")
        core.advance_pace_trim(min_clear)
        lap_num += 1
        lap_start_time = time_elapsed
        lap_start_dist = dist
        min_clear = 999.0
        max_speed = 0.0
        if lap_num > 10:
            # Verify Active Steered Cooldown Braking after Lap 10 completion
            cooldown_min_clear = 999.0
            cooldown_steps = 0
            for c_step in range(350):
                scans = obs["scans"][0]
                vx = obs["linear_vels_x"][0]
                cooldown_min_clear = min(cooldown_min_clear, float(np.min(scans)))
                if vx <= 0.05:
                    break
                steer, _ = core.step(scans, -2.35, 4.7/1080.0, vx)
                obs, _, _, _ = env.step(np.array([[steer, 0.0]]))
                cooldown_steps += 1
                if env.collisions[0] == 1.0:
                    collisions += 1
                    print(f"COLLISION during post-Lap-10 cooldown at step {c_step}!")
                    break
            print(f"Post-Lap-10 Active Steered Cooldown: halted at {obs['linear_vels_x'][0]:.2f} m/s in {cooldown_steps*0.01:.2f}s (MinClear={cooldown_min_clear:.3f}m, Collisions={collisions})")
            break

cum_saved = sum([max(0.0, x[6]) for x in laps])
max_gain = max([x[6] for x in laps])

print("\n" + "="*94)
print("  REAL-TIME LIDAR RACING CONTROLLER (V2 ADAPTIVE PACE) - 10-LAP BENCHMARK & TIME-GAIN REPORT")
print("="*94)
print(f"  +---------+--------+--------------+---------------+--------------+--------------+------------+")
print(f"  |  Lap #  |  Trim  |   Lap Time   | Gain vs Lap 1 |  Delta Prev  |  Peak Speed  | Min Clear. |")
print(f"  +---------+--------+--------------+---------------+--------------+--------------+------------+")
for l, t, d, s, c, tr, g1, gp in laps:
    star = " <-- FASTEST LAP ★" if t == min([x[1] for x in laps]) else ""
    print(f"  |  Lap {l:2d} |   {tr}   |    {t:6.3f} s  |    {g1:+6.3f} s   |   {gp:+6.3f} s   |   {s:5.2f} m/s  |   {c:5.3f} m  |{star}")
print(f"  +---------+--------+--------------+---------------+--------------+--------------+------------+")

flying_laps = [x[1] for x in laps[1:]]
print(f"  ⏱️  FASTEST LAP TIME       : {min([x[1] for x in laps]):.3f} s (Lap {laps[[x[1] for x in laps].index(min([x[1] for x in laps]))][0]}, Max Gain: {max_gain:+.3f} s)")
print(f"  ⚡ TOTAL TIME SAVED       : {cum_saved:+.3f} s gained across 10 laps vs Lap 1 reference")
print(f"  📊 FLYING LAP AVERAGE     : {np.mean(flying_laps):.3f} s")
print(f"  📊 OVERALL LAP AVERAGE    : {np.mean([x[1] for x in laps]):.3f} s")
print(f"  🏁 TOTAL RACE TIME        : {sum([x[1] for x in laps]):.3f} s")
print(f"  🚀 PEAK SPEED ACHIEVED    : {max([x[3] for x in laps]):.2f} m/s")
print(f"  🛡️  MIN WALL CLEARANCE     : {min([x[4] for x in laps]):.3f} m ({collisions} Collisions)")

csv_path = "lap_times_sakhir_v2_fast.csv"
gains_csv_path = "lap_time_gains_sakhir.csv"

with open(csv_path, "w", newline="") as f, open(gains_csv_path, "w", newline="") as fg:
    writer = csv.writer(f)
    gwriter = csv.writer(fg)
    writer.writerow([
        "lap_number", "pace_trim", "lap_time_sec", "time_gained_vs_lap1_sec",
        "delta_vs_prev_lap_sec", "cumulative_time_saved_sec", "best_lap_sec",
        "avg_lap_sec", "lap_distance_m", "max_speed_mps",
        "min_wall_clearance_m", "total_elapsed_sec"
    ])
    gwriter.writerow([
        "lap_number", "pace_trim", "lap_time_sec", "time_gained_vs_lap1_sec",
        "delta_vs_prev_lap_sec", "cumulative_time_saved_sec",
        "trim_min_speed_mps", "trim_max_speed_mps", "trim_a_lat_max",
        "min_wall_clearance_m"
    ])
    tot_time = 0.0
    cum_s = 0.0
    best_t = float("inf")
    all_times = [x[1] for x in laps]
    for idx, (l, t, d, s, c, tr, g1, gp) in enumerate(laps, 1):
        tot_time += t
        cum_s += max(0.0, g1)
        best_t = min(best_t, t)
        avg_t = np.mean(all_times[:idx])
        writer.writerow([
            l, tr, f"{t:.3f}", f"{g1:+.3f}", f"{gp:+.3f}", f"{cum_s:+.3f}",
            f"{best_t:.3f}", f"{avg_t:.3f}", f"{d:.2f}", f"{s:.2f}", f"{c:.3f}", f"{tot_time:.3f}"
        ])
        trim_num = int(tr.replace("T", ""))
        tp = PACE_TRIMS[trim_num]
        gwriter.writerow([
            l, tr, f"{t:.3f}", f"{g1:+.3f}", f"{gp:+.3f}", f"{cum_s:+.3f}",
            f"{tp['min_speed']:.2f}", f"{tp['max_speed']:.2f}", f"{tp['a_lat_max']:.2f}", f"{c:.3f}"
        ])

print(f"  📁 MAIN CSV SAVED TO      : {os.path.abspath(csv_path)}")
print(f"  📁 TIME-GAINS CSV SAVED TO: {os.path.abspath(gains_csv_path)}")
print("="*94)
