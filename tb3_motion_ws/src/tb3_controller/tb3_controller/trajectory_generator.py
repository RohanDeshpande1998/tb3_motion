#!/usr/bin/env python3
import numpy as np
from scipy.interpolate import splprep, splev, interp1d
import os
import sys
import yaml 
import math
import logging

def generate_bspline(control_points, num_points=1000, degree=3):
    x, y = control_points[:, 0], control_points[:, 1]
    tck, _ = splprep([x, y], s=0, k=degree)
    u_fine = np.linspace(0, 1, num_points)
    x_fine, y_fine = splev(u_fine, tck)
    return tck, u_fine, x_fine, y_fine

def compute_arc_length(x, y):
    dx, dy = np.gradient(x), np.gradient(y)
    ds = np.sqrt(dx**2 + dy**2)
    s = np.insert(np.cumsum(ds[:-1]), 0, 0)
    return s

def trapezoidal_profile(total_length, v_max, a_max, dt=0.01):
    t_acc = v_max / a_max
    d_acc = 0.5 * a_max * t_acc**2

    if 2 * d_acc > total_length:
        d_acc = total_length / 2
        t_acc = np.sqrt(2 * d_acc / a_max)
        t_flat = 0
    else:
        d_flat = total_length - 2 * d_acc
        t_flat = d_flat / v_max

    t_total = 2 * t_acc + t_flat
    t_samples = np.arange(0, t_total, dt)
    s_profile = []

    for t in t_samples:
        if t < t_acc:
            s_t = 0.5 * a_max * t**2
        elif t < t_acc + t_flat:
            s_t = d_acc + v_max * (t - t_acc)
        else:
            t_dec = t - t_acc - t_flat
            s_t = d_acc + d_flat + v_max * t_dec - 0.5 * a_max * t_dec**2
        s_profile.append(s_t)

    s_profile = np.clip(s_profile, 0, total_length)
    return np.array(t_samples), np.array(s_profile)

def main():
    if len(sys.argv) < 5:
        print("Usage: generate_trajectory.py <x> <y> <yaw> <yaml_file>")
        sys.exit(1)
        
    x0 = float(sys.argv[1])
    y0 = float(sys.argv[2])
    yaw0 = float(sys.argv[3])
    yaml_path = sys.argv[4]
    with open(yaml_path, 'r') as f:
        config = yaml.safe_load(f)
    waypoints = np.array(config['waypoints'])

    # Insert initial pose as first control point
    control_points = np.vstack(([x0, y0], waypoints))

    # === NEW: Add "ghost" point behind initial pose ===
    ghost_dist = 0.01  # meters (tune this)
    x_ghost = x0 - ghost_dist * math.cos(yaw0)
    y_ghost = y0 - ghost_dist * math.sin(yaw0)
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger("trajectory_generator")

    logger.warning(f"x,y=({x_ghost}, {y_ghost})")
    # === Update control points: ghost → start → waypoints ===
    control_points = np.vstack(([x_ghost, y_ghost], [x0, y0], waypoints))
    
    v_max = 0.2
    a_max = 0.1
    dt = 0.05

    tck, u_fine, x_fine, y_fine = generate_bspline(control_points)
    s_arc = compute_arc_length(x_fine, y_fine)
    
    # Step 1: Find closest point on spline to actual robot pose
    dists = np.sqrt((x_fine - x0)**2 + (y_fine - y0)**2)
    start_idx = np.argmin(dists)
        
    # Step 2: Trim spline from that point onward
    x_fine_trimmed = x_fine[start_idx:]
    y_fine_trimmed = y_fine[start_idx:]
    s_arc_trimmed = compute_arc_length(x_fine_trimmed, y_fine_trimmed)

    total_length = s_arc_trimmed[-1]

    t_samples, s_profile = trapezoidal_profile(total_length, v_max, a_max, dt)
    interp_x = interp1d(s_arc_trimmed, x_fine_trimmed, bounds_error=False, fill_value="extrapolate")
    interp_y = interp1d(s_arc_trimmed, y_fine_trimmed, bounds_error=False, fill_value="extrapolate")
    x_traj = interp_x(s_profile)
    y_traj = interp_y(s_profile)

    output_dir = os.path.join(os.path.expanduser('~'), 'tb3_motion_ws', 'data')
    os.makedirs(output_dir, exist_ok=True)

    trajectory_file = os.path.join(output_dir, 'trajectory.npz')
    np.savez(trajectory_file, x=x_traj, y=y_traj, dt=dt)
    print("Trajectory saved to data/trajectory.npz")

if __name__ == "__main__":
    main()

