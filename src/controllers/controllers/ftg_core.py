"""
Pure-Python / NumPy core of the reactive Follow-the-Gap controller.

This module has NO ROS dependency, so the exact same code is used by:
  * the ROS 2 node (gap_node.py), and
  * the offline benchmark / tuning scripts (direct f1tenth_gym simulation).

Supported algorithm variants (selected with `mode`):
  'ftg_pd'    Follow-the-Gap + asymmetrical racing-line disparity extender +
              depth-weighted gap scoring + apex-biased convolved target +
              smoothed PD steering + physics-informed trail-braking & corner-exit
              lookahead speed profile.
  'ftg_pp'    Follow-the-Gap + disparity extender, pure-pursuit steering law
              (delta = atan(2 L sin(alpha) / Ld)) towards the gap target point.
  'disparity' Disparity-Extender gap follower: no bubble / max-gap search,
              drives towards the deepest reachable beam after disparity extension.
"""

import numpy as np


DEFAULTS = dict(
    mode='ftg_pd',
    # --- perception (SpeedMaster racing-line tuned & 10-lap collision-free verified) ---
    preprocess_conv_size=5,
    max_lidar_dist=29.0,
    disparity_thresh=0.3087,
    car_width_safety=0.2911,
    inside_width_scale=0.98,      # asymmetrical disparity scaling for apex tuck
    fov_angle_deg=140.0,
    bubble_radius=0.4873,
    gap_thresh=1.5,
    best_point_conv_size=96,
    use_weighted_center=0.06560953051188756,   # smooth apex target
    gap_depth_weight=0.1887,      # score gaps by length * (mean_depth ** gap_depth_weight)
    # --- steering ---
    steer_gain=0.7709,            # ftg_pd / disparity proportional gain
    steer_deriv_gain=0.1716,      # ftg_pd derivative gain (per new unique scan)
    pp_lookahead_k=0.30,          # ftg_pp: Ld = clip(k * v, Ld_min, Ld_max)
    pp_lookahead_min=1.5,
    pp_lookahead_max=5.0,
    steer_smoothing=0.1089,       # low-pass filter on steering command
    # --- speed (early trail-braking + corner-exit lookahead boost + straightaway deadband) ---
    max_speed=18.0,               # high-speed straights unleashed!
    min_speed=6.10,               # fast, stable corner rotation
    a_brake=7.95,
    a_lat_max=13.0,
    safety_margin=1.12,
    front_cone_half=17,
    exit_lookahead_blend=0.22,    # blends forward nose range with deepest exit-ray range
    brake_entry_gain=0.14,        # early trail-braking proportional to corner closing rate * v
    deadband_rad=0.038,           # 2.18° straightaway deadband on v_lat (prevents straightaway throttling)
    narrow_corridor_width=0.0,    # 0.0 disables indoor hallway mode for race tracks like Sakhir
)

WHEELBASE = 0.3302
MAX_STEER = 0.4189


class FollowGapCore:
    def __init__(self, **params):
        p = dict(DEFAULTS)
        unknown = set(params) - set(p)
        if unknown:
            raise ValueError(f'Unknown parameters: {sorted(unknown)}')
        p.update(params)
        self.p = p
        self.prev_target_angle = 0.0
        self.prev_steer = 0.0
        self.prev_front = 25.0
        self.last_scan = None
        self.last_cmd = (0.0, 0.0)
        self.last_debug = {}
        self._n = None

    # ------------------------------------------------------------------ setup
    def _setup(self, n, angle_min, angle_inc):
        self._n = n
        self.angle_min = angle_min
        self.angle_inc = angle_inc
        self.angles = angle_min + np.arange(n) * angle_inc
        fov = np.deg2rad(self.p['fov_angle_deg'] / 2.0)

        def idx(a):
            return int(np.clip(round((a - angle_min) / angle_inc), 10, n - 11))

        self.min_idx = int(np.clip(np.ceil((-fov - angle_min) / angle_inc), 0, n - 1))
        self.max_idx = int(np.clip(np.floor((fov - angle_min) / angle_inc), 1, n))
        self.l90, self.r90 = idx(np.pi / 2), idx(-np.pi / 2)
        self.l60, self.r60 = idx(np.deg2rad(60)), idx(np.deg2rad(-60))
        self.center_idx = n // 2

    # ----------------------------------------------------------- perception
    def preprocess_lidar(self, ranges):
        p = self.p
        proc = np.nan_to_num(np.asarray(ranges, dtype=np.float64),
                             nan=0.0, posinf=p['max_lidar_dist'], neginf=0.0)
        k = p['preprocess_conv_size']
        if k > 1:
            proc = np.convolve(proc, np.ones(k) / k, mode='same')
        return np.clip(proc, 0.0, p['max_lidar_dist'])

    def apply_disparity_extender(self, r):
        p = self.p
        out = r.copy()
        jumps = np.where(np.abs(np.diff(r)) > p['disparity_thresh'])[0]
        for i in jumps:
            if r[i] < r[i + 1]:          # obstacle edge on the right side of the jump
                d = max(r[i], 0.1)
                w = p['car_width_safety'] * (p['inside_width_scale'] if (i > self.center_idx) else 1.0)
                nb = int(np.ceil(np.arctan2(w, d) / self.angle_inc))
                e = min(len(out), i + 1 + nb)
                out[i + 1:e] = np.minimum(out[i + 1:e], d)
            else:
                d = max(r[i + 1], 0.1)
                w = p['car_width_safety'] * (p['inside_width_scale'] if (i < self.center_idx) else 1.0)
                nb = int(np.ceil(np.arctan2(w, d) / self.angle_inc))
                s = max(0, i + 1 - nb)
                out[s:i + 1] = np.minimum(out[s:i + 1], d)
        return out

    def find_max_gap(self, free):
        p = self.p
        for th in (p['gap_thresh'], 0.5, 0.0):
            is_free = free > th
            if np.any(is_free):
                break
        padded = np.concatenate(([False], is_free, [False]))
        starts = np.where(padded[1:] & ~padded[:-1])[0]
        ends = np.where(~padded[1:] & padded[:-1])[0]
        if len(starts) == 0:
            c = len(free) // 2
            return c - 1, c + 1
        lengths = (ends - starts).astype(np.float64)
        w = p['gap_depth_weight']
        if w > 0.0:
            depth = np.array([free[s:e].mean() for s, e in zip(starts, ends)])
            score = lengths * (depth ** w)
        else:
            score = lengths
        k = int(np.argmax(score))
        return int(starts[k]), int(ends[k])

    def find_best_point(self, s, e, r):
        p = self.p
        g = r[s:e]
        if len(g) == 0:
            return len(r) // 2
        k = p['best_point_conv_size']
        if len(g) >= k:
            peak = s + int(np.argmax(np.convolve(g, np.ones(k) / k, mode='same')))
        else:
            peak = s + int(np.argmax(g))
        lam = p['use_weighted_center']
        best = int(round((1 - lam) * peak + lam * 0.5 * (s + e)))
        return int(np.clip(best, s, max(s, e - 1)))

    # --------------------------------------------------------------- control
    def step(self, ranges, angle_min, angle_inc, current_vel):
        """Return (steering_angle [rad], speed [m/s]) for one LiDAR scan."""
        p = self.p
        raw = np.nan_to_num(np.asarray(ranges, dtype=np.float64),
                            nan=0.0, posinf=30.0, neginf=0.0)
        if self._n != len(raw):
            self._setup(len(raw), angle_min, angle_inc)

        # The ROS bridge re-publishes the same scan at 250 Hz between 100 Hz physics
        # steps; reuse the previous command so the derivative term stays consistent.
        if self.last_scan is not None and np.array_equal(raw, self.last_scan):
            return self.last_cmd
        self.last_scan = raw

        # Local corridor width (min of 60 and 90 deg side sectors)
        left = min(np.median(raw[self.l90 - 10:self.l90 + 10]), np.median(raw[self.l60 - 10:self.l60 + 10]))
        right = min(np.median(raw[self.r90 - 10:self.r90 + 10]), np.median(raw[self.r60 - 10:self.r60 + 10]))
        corridor = left + right
        narrow = corridor < p['narrow_corridor_width']

        proc = self.apply_disparity_extender(self.preprocess_lidar(raw))
        fov = np.zeros_like(proc)
        fov[self.min_idx:self.max_idx] = proc[self.min_idx:self.max_idx]

        ci = self.min_idx + int(np.argmin(proc[self.min_idx:self.max_idx]))
        closest = max(fov[ci], 0.1)

        if p['mode'] == 'disparity':
            k = max(3, p['best_point_conv_size'] // 2)
            sm = np.convolve(fov, np.ones(k) / k, mode='same')
            sm[:self.min_idx] = 0.0
            sm[self.max_idx:] = 0.0
            best = int(np.argmax(sm))
        else:
            bub = p['bubble_radius'] if not narrow else min(p['bubble_radius'], 0.30)
            nb = int(np.ceil(np.arctan2(bub, closest) / self.angle_inc))
            fov[max(self.min_idx, ci - nb):min(self.max_idx, ci + nb + 1)] = 0.0
            s, e = self.find_max_gap(fov)
            best = self.find_best_point(s, e, fov)

        alpha = float(self.angles[best])
        # Prevent discontinuous gap jumps across sides when heavily cornering
        if abs(alpha - self.prev_target_angle) > 1.30 and abs(self.prev_steer) > 0.15:
            alpha = float(np.clip(alpha, self.prev_target_angle - 0.40, self.prev_target_angle + 0.40))

        # ----- steering law
        if p['mode'] == 'ftg_pp':
            ld = float(np.clip(p['pp_lookahead_k'] * current_vel,
                               p['pp_lookahead_min'], p['pp_lookahead_max']))
            ld = min(ld, max(proc[best], 0.5))
            steer = np.arctan2(2.0 * WHEELBASE * np.sin(alpha), ld)
        else:
            if narrow and abs(alpha) > 0.25:
                g = 1.35
            else:
                g = float(np.clip(1.0 - 0.03 * max(0.0, current_vel - 6.0), 0.55, 1.0))
            steer = p['steer_gain'] * g * alpha + p['steer_deriv_gain'] * (alpha - self.prev_target_angle)
        a = p['steer_smoothing']
        if a > 0.0:
            steer = (1 - a) * steer + a * self.prev_steer
        steer = float(np.clip(steer, -MAX_STEER, MAX_STEER))
        self.prev_target_angle = alpha
        self.prev_steer = steer

        # ----- speed profile (early trail-braking + corner-exit lookahead boost)
        c, h = self.center_idx, p['front_cone_half']
        front = float(np.percentile(proc[c - h:c + h], 25))
        si = int(np.clip(c + steer / self.angle_inc, self.min_idx, self.max_idx - 1))
        sdist = float(np.percentile(proc[max(0, si - 15):si + 15], 25))
        tdist = float(np.percentile(proc[max(0, best - 12):min(len(proc), best + 12)], 30))

        closing_rate = max(0.0, self.prev_front - front)
        self.prev_front = front

        if narrow:
            vmin, vmax, ab, al, margin = 1.8, 6.0, 2.8, 2.5, 2.8
            d_avail = max(0.0, min(front, 1.1 * sdist) - margin)
            eff = max(abs(steer), 0.75 * abs(alpha))
            wall = float(np.clip((closest - 0.25) / 0.5, 0.5, 1.0))
        else:
            vmin, vmax, ab, al, margin = (
                p['min_speed'], p['max_speed'], p['a_brake'], p['a_lat_max'], p['safety_margin']
            )
            dynamic_margin = margin + p['brake_entry_gain'] * closing_rate * current_vel
            if abs(alpha) < 0.25 and abs(steer) < 0.22:
                blend = p['exit_lookahead_blend']
                eff_front = (1.0 - blend) * front + blend * min(tdist, sdist * 1.25)
            else:
                eff_front = front
            d_avail = max(0.0, min(eff_front, 1.15 * sdist) - dynamic_margin)
            eff = max(abs(steer), 0.72 * abs(alpha))
            wall = float(np.clip((closest - 0.22) / 0.45, 0.55, 1.0))

        v_brake = np.sqrt(vmin ** 2 + 2.0 * ab * d_avail)
        if narrow:
            v_lat = np.sqrt(al / (np.tan(min(eff, MAX_STEER)) / WHEELBASE)) if eff > 0.015 else vmax
        else:
            curv_eff = max(0.0, eff - p.get('deadband_rad', 0.040))
            v_lat = np.sqrt(al / (np.tan(min(curv_eff, MAX_STEER)) / WHEELBASE)) if curv_eff > 0.01 else vmax
        speed = float(max(vmin, min(vmax, v_brake, v_lat) * wall))

        self.last_cmd = (steer, speed)
        self.last_debug = dict(best_idx=best, best_dist=float(proc[best]), target_angle=alpha,
                               closest=closest, corridor=corridor)
        return self.last_cmd
