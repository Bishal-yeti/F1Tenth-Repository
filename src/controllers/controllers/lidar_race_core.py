"""
Real-Time LiDAR Dynamic Racing-Line Core (Version 2: Qualifying Edition)

This module implements a pure perception-driven, physics-informed racing controller
designed to maximize single-lap pace and apex-hugging efficiency using real LiDAR data.
"""

from controllers.ftg_core import FollowGapCore, DEFAULTS

DEFAULTS_V2 = dict(DEFAULTS)
DEFAULTS_V2.update(
    mode='ftg_pd',
    fov_angle_deg=140.0,
    bubble_radius=0.4873,
    gap_thresh=1.5,
    best_point_conv_size=96,
    use_weighted_center=0.0656,     # Apex-biased goal selection
    gap_depth_weight=0.1887,
    steer_gain=0.7709,
    steer_deriv_gain=0.1716,
    steer_smoothing=0.1089,
    max_speed=18.0,                 # High-speed straightaway limit
    min_speed=6.10,                 # High corner rotation velocity
    a_brake=7.95,                   # Responsive kinematic braking
    a_lat_max=13.0,                 # Peak lateral tire grip
    safety_margin=1.12,
    front_cone_half=17,             # Tight forward corridor cone (34 beams)
    exit_lookahead_blend=0.22,      # Corner-exit throttle acceleration blend
    brake_entry_gain=0.14,          # Trail-braking closing-rate gain
    deadband_rad=0.038,             # 2.18° straightaway deadband
    inside_width_scale=0.98,        # Dynamic apex tuck scale
    narrow_corridor_width=0.0       # Disabled indoor hallway mode for race circuits
)


class RealLidarRaceCore(FollowGapCore):
    """
    Real-Time LiDAR Dynamic Racing Core (Version 2).
    Inherits from FollowGapCore and specializes perception and control
    parameters for championship-level apex tracking and maximum velocity.
    """
    def __init__(self, **params):
        p = dict(DEFAULTS_V2)
        p.update(params)
        super().__init__(**p)
