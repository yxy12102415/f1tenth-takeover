"""Dynamic bicycle MPC following the current Frenet-selected local path."""
import math
import time
import numpy as np
from native_dynamic_mpc import solve

CONTROLLER_NAME = 'closed_loop_frenet_dynamic_mpc'
CONTROLLER_LABEL = 'Frenet+dynamic-MPC'
CONFIG = {
    'type': 'dynamic_mpc', 'model': 'linear_dynamic_bicycle',
    'horizon': 16, 'prediction_dt_s': 0.05,
    'states': ['cross_track_error', 'heading_error', 'lateral_speed', 'yaw_rate'],
    'cross_track_weight': 140.0, 'terminal_cross_track_weight': 210.0,
    'heading_weight': 70.0, 'terminal_heading_weight': 105.0,
    'lateral_speed_weight': 1.0, 'terminal_lateral_speed_weight': 1.5,
    'yaw_rate_weight': 3.0, 'terminal_yaw_rate_weight': 4.5,
    'steering_weight': 1.2, 'steering_rate_weight': 16.0,
    'steering_rate_limit_rad_s': 1.0, 'iterations': 80,
    'maximum_cross_track_error_m': 0.25,
    'maximum_heading_error_rad': 0.70,
    'maximum_lateral_speed_mps': 2.5, 'maximum_yaw_rate_rad_s': 8.0,
    'minimum_model_speed_mps': 0.50, 'maximum_model_speed_mps': 1.50,
    'mass_kg': 3.74, 'yaw_inertia_kg_m2': 0.04712,
    'front_length_m': 0.12, 'rear_length_m': 0.13,
    'front_cornering_stiffness_n_rad': 90.0,
    'rear_cornering_stiffness_n_rad': 96.0,
    'discretization': 'backward_euler', 'iteration_backend': 'native_dynamic_v1',
    'state_source': 'vesc_odometry_vy_zero_yaw_estimated_from_speed_and_servo',
}


def model_parameters():
    return [CONFIG[key] for key in (
        'mass_kg', 'yaw_inertia_kg_m2', 'front_length_m', 'rear_length_m',
        'front_cornering_stiffness_n_rad', 'rear_cornering_stiffness_n_rad')]


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def path_geometry(path, x, y):
    path = np.asarray(path, dtype=float)
    # Two terminal samples still define a segment and a first-order yaw
    # gradient. A singleton is handled by the controller's endpoint logic.
    if path.ndim != 2 or path.shape[0] < 2 or path.shape[1] < 3:
        raise ValueError('tracker_path_invalid')
    if not np.all(np.isfinite(path[:, :3])):
        raise ValueError('tracker_path_non_finite')
    nearest = int(np.argmin(np.hypot(path[:, 0] - x, path[:, 1] - y)))
    segment = np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1)
    if np.any(segment < 1e-5):
        segment = np.maximum(segment, 1e-5)
    cumulative = np.r_[0.0, np.cumsum(segment)]
    yaw = np.unwrap(path[:, 2])
    curvature = np.gradient(yaw, cumulative, edge_order=1)
    curvature = np.clip(curvature, -math.tan(0.27) / 0.25,
                        math.tan(0.27) / 0.25)
    return path, nearest, cumulative, curvature


def project_reference(path, arc, x, y):
    """Continuous reference pose on the closest nondegenerate segment."""
    edges = np.diff(path[:, :2], axis=0)
    lengths_squared = np.sum(edges*edges, axis=1)
    valid = lengths_squared > 1e-10
    if not np.any(valid):
        raise ValueError('tracker_path_invalid')
    fractions = np.clip(np.sum((np.array([x, y])-path[:-1, :2])*edges, axis=1)
                        / np.maximum(lengths_squared, 1e-10), 0., 1.)
    points = path[:-1, :2] + fractions[:, None]*edges
    distance = np.sum((points-np.array([x, y]))**2, axis=1)
    distance[~valid] = np.inf
    index = int(np.argmin(distance))
    fraction = fractions[index]
    yaw = np.unwrap(path[:, 2])
    reference = np.r_[points[index], yaw[index]+fraction*(yaw[index+1]-yaw[index])]
    reference_s = arc[index]+fraction*(arc[index+1]-arc[index])
    return reference, float(reference_s), index


class LocalPathTracker:
    def __init__(self, steering_limit=0.269):
        self.steering_limit = float(steering_limit)
        self.angle = 0.0
        self.previous = np.zeros(CONFIG['horizon'], dtype=float)
        self.last = {}

    def reset(self):
        self.angle = 0.0
        self.previous[:] = 0.0
        self.last = {}

    def compute(self, path, x, y, heading, dt, speed_mps=0.75,
                lateral_speed_mps=0.0, yaw_rate_rad_s=0.0):
        started = time.perf_counter()
        if not all(math.isfinite(value) for value in
                   (x, y, heading, dt, speed_mps, lateral_speed_mps, yaw_rate_rad_s)):
            raise ValueError('mpc_non_finite')
        if (abs(lateral_speed_mps) > CONFIG['maximum_lateral_speed_mps']
                or abs(yaw_rate_rad_s) > CONFIG['maximum_yaw_rate_rad_s']):
            raise ValueError('dynamic_state_out_of_range')
        path, nearest, arc, curvature = path_geometry(path, x, y)
        reference, reference_s, nearest = project_reference(path, arc, x, y)
        lateral = (-math.sin(reference[2])*(x-reference[0])
                   + math.cos(reference[2])*(y-reference[1]))
        heading_error = wrap(heading-reference[2])
        if abs(lateral) > CONFIG['maximum_cross_track_error_m']:
            raise ValueError('mpc_cross_track_error')
        if abs(heading_error) > CONFIG['maximum_heading_error_rad']:
            raise ValueError('mpc_heading_error')
        horizon = CONFIG['horizon']; predict_dt = CONFIG['prediction_dt_s']
        vref = float(np.clip(abs(speed_mps), CONFIG['minimum_model_speed_mps'],
                            CONFIG['maximum_model_speed_mps']))
        speeds = np.full(horizon, vref)
        preview_s = reference_s+np.arange(horizon)*vref*predict_dt
        preview_curvature = np.interp(preview_s, arc, curvature)
        control_dt = max(0.0, min(0.05, float(dt)))
        first_step = CONFIG['steering_rate_limit_rad_s']*control_dt
        weights = [CONFIG[key] for key in (
            'cross_track_weight', 'heading_weight', 'lateral_speed_weight', 'yaw_rate_weight',
            'terminal_cross_track_weight', 'terminal_heading_weight',
            'terminal_lateral_speed_weight', 'terminal_yaw_rate_weight',
            'steering_weight', 'steering_rate_weight')]
        control, maxima, iterations = solve(
            [lateral, heading_error, lateral_speed_mps, yaw_rate_rad_s],
            speeds, preview_curvature, model_parameters(), weights, self.previous,
            self.angle, self.steering_limit, first_step,
            CONFIG['steering_rate_limit_rad_s']*predict_dt, predict_dt, CONFIG['iterations'])
        limits = np.array([CONFIG['maximum_cross_track_error_m'],
                           CONFIG['maximum_heading_error_rad'],
                           CONFIG['maximum_lateral_speed_mps'],
                           CONFIG['maximum_yaw_rate_rad_s']])
        if not np.isfinite(control).all() or not np.isfinite(maxima).all():
            raise ValueError('mpc_non_finite')
        if np.any(maxima > limits):
            raise ValueError('mpc_infeasible')
        self.previous = control
        self.angle = float(control[0])
        self.last = {
            'nearest_local_index': nearest, 'reference_arc_m': reference_s,
            'reference_heading_rad': float(reference[2]),
            'cross_track_error_m': lateral, 'heading_error_rad': heading_error,
            'lateral_speed_mps': float(lateral_speed_mps),
            'yaw_rate_rad_s': float(yaw_rate_rad_s),
            'measured_speed_mps': float(speed_mps), 'prediction_speed_mps': vref,
            'model': CONFIG['model'], 'state_dimension': 4,
            'state_source': CONFIG['state_source'],
            'control_dt_s': control_dt, 'first_move_limit_rad': first_step,
            'prediction_distance_m': horizon*vref*predict_dt,
            'mpc_iterations': iterations,
            'mpc_solve_ms': (time.perf_counter()-started)*1000.0,
            'mpc_predicted_max_cross_track_m': float(maxima[0]),
            'mpc_predicted_max_heading_rad': float(maxima[1]),
            'mpc_predicted_max_lateral_speed_mps': float(maxima[2]),
            'mpc_predicted_max_yaw_rate_rad_s': float(maxima[3]),
        }
        return self.angle
