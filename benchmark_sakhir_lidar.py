import numpy as np
import gym, f110_gym, yaml, sys, os, csv
sys.path.insert(0, os.path.abspath("src/controllers"))
from controllers.lidar_race_core import RealLidarRaceCore

with open("src/f1tenth_gym_ros/config/sim.yaml") as f:
    conf = yaml.safe_load(f)["bridge"]["ros__parameters"]

map_path = conf["map_path"]
map_ext = conf["map_img_ext"]
sx, sy, stheta = float(conf["sx"]), float(conf["sy"]), float(conf["stheta"])
print(f"Map: {map_path}, Start: ({sx}, {sy}, {stheta})")

env = gym.make("f110_gym:f110-v0", map=map_path, map_ext=map_ext, num_agents=1)
obs, _, _, _ = env.reset(np.array([[sx, sy, stheta]]))

core = RealLidarRaceCore()

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
    
    steer, speed = core.step(scans, -2.35, 4.7/1080.0, vx)
    obs, _, done, info = env.step(np.array([[steer, speed]]))
    
    if env.collisions[0] == 1.0:
        collisions += 1
        print(f"COLLISION at step {step} (t={time_elapsed:.2f}s, dist={dist:.1f}m, pos=({px:.2f}, {py:.2f}), min_clear={min_clear:.3f}m)")
        break
        
    yaw_diff = abs(np.arctan2(np.sin(pth - stheta), np.cos(pth - stheta)))
    # On Sakhir (track length ~440m), check lap completion
    if cur_lap_dist > 400.0 and d_to_start < 2.5 and yaw_diff < 1.0:
        laps.append((lap_num, cur_lap_time, cur_lap_dist, max_speed, min_clear))
        print(f"Lap {lap_num:2d} finished: {cur_lap_time:6.3f}s | Dist: {cur_lap_dist:6.2f}m | Peak: {max_speed:5.2f}m/s | MinClear: {min_clear:5.3f}m")
        lap_num += 1
        lap_start_time = time_elapsed
        lap_start_dist = dist
        min_clear = 999.0
        max_speed = 0.0
        if lap_num > 10:
            break

print("\n" + "="*78)
print("  REAL-TIME LIDAR RACING CONTROLLER (VERSION 2) - 10-LAP OFFICIAL BENCHMARK")
print("="*78)
print(f"  +---------+--------------+-------------+--------------+------------+")
print(f"  |  Lap #  |   Lap Time   |  Distance   |  Peak Speed  | Min Clear. |")
print(f"  +---------+--------------+-------------+--------------+------------+")
for l, t, d, s, c in laps:
    star = " <-- FASTEST LAP ★" if t == min([x[1] for x in laps]) else ""
    print(f"  |  Lap {l:2d} |    {t:6.3f} s  |   {d:6.2f} m  |   {s:5.2f} m/s  |   {c:5.3f} m  |{star}")
print(f"  +---------+--------------+-------------+--------------+------------+")

flying_laps = [x[1] for x in laps[1:]]
print(f"  ⏱️  FASTEST LAP TIME     : {min([x[1] for x in laps]):.3f} s (Lap {laps[[x[1] for x in laps].index(min([x[1] for x in laps]))][0]})")
print(f"  📊 FLYING LAP AVERAGE   : {np.mean(flying_laps):.3f} s (±{np.std(flying_laps):.3f} s)")
print(f"  📊 OVERALL LAP AVERAGE  : {np.mean([x[1] for x in laps]):.3f} s")
print(f"  🏁 TOTAL RACE TIME      : {sum([x[1] for x in laps]):.3f} s")
print(f"  🚀 PEAK SPEED ACHIEVED  : {max([x[3] for x in laps]):.2f} m/s")
print(f"  🛡️  MIN WALL CLEARANCE   : {min([x[4] for x in laps]):.3f} m ({collisions} Collisions)")
csv_path = "lap_times_sakhir_v2_fast.csv"
with open(csv_path, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow([
        "lap_number", "lap_time_sec", "best_lap_sec", "avg_lap_sec",
        "lap_distance_m", "max_speed_mps", "min_wall_clearance_m", "total_elapsed_sec"
    ])
    tot_time = 0.0
    best_t = float("inf")
    all_times = [x[1] for x in laps]
    for idx, (l, t, d, s, c) in enumerate(laps, 1):
        tot_time += t
        best_t = min(best_t, t)
        avg_t = np.mean(all_times[:idx])
        writer.writerow([l, f"{t:.3f}", f"{best_t:.3f}", f"{avg_t:.3f}", f"{d:.2f}", f"{s:.2f}", f"{c:.3f}", f"{tot_time:.3f}"])
print(f"  📁 CSV RESULTS SAVED TO : {os.path.abspath(csv_path)}")
print("="*78)
