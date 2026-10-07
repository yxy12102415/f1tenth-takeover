"""Pure-Pursuit geometry and explicit laser/base pose contracts.

Reference: old pure_pursuit_node.cpp uses curvature=2*y/L**2. Here the actual
target chord length and atan(wheelbase*curvature) are used, with open-route
gates/limits inherited from the separately validated migration core.
"""
import math
import numpy as np
from f1tenth_safety_migration.route_mpc_core import OpenRouteMpcCore, RouteMpcConfig


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_yaw(q):
    values = np.array([q.x, q.y, q.z, q.w], dtype=float)
    if not np.isfinite(values).all() or abs(float(values @ values)-1.) > .02:
        raise ValueError('invalid quaternion')
    if abs(q.x) > .01 or abs(q.y) > .01:
        raise ValueError('non-planar pose')
    return 2*math.atan2(q.z, q.w)


def laser_to_base(pose, covariance, offset=.27):
    """Planar, zero-yaw laser extrinsic: base->laser x=offset, y=0."""
    x, y, yaw = pose
    covariance = np.asarray(covariance, dtype=float).reshape(3, 3)
    if not np.isfinite([x,y,yaw,offset]).all() or not np.isfinite(covariance).all():
        raise ValueError('nonfinite pose/covariance')
    jacobian = np.array([[1., 0., offset*math.sin(yaw)],
                         [0., 1., -offset*math.cos(yaw)], [0.,0.,1.]])
    return np.array([x-offset*math.cos(yaw), y-offset*math.sin(yaw), wrap(yaw)]), jacobian @ covariance @ jacobian.T


def particle_estimate(particles, weights):
    if not np.isfinite(particles).all() or not np.isfinite(weights).all():
        raise ValueError('nonfinite particles')
    if np.any(weights < 0.) or float(np.sum(weights)) <= 0.:
        raise ValueError('invalid particle weights')
    weights = weights / np.sum(weights)
    xy = weights @ particles[:, :2]
    sine, cosine = weights @ np.sin(particles[:,2]), weights @ np.cos(particles[:,2])
    if math.hypot(sine, cosine) < .1:
        raise ValueError('ambiguous heading')
    yaw = math.atan2(sine, cosine)
    residual = particles - [xy[0], xy[1], yaw]
    residual[:,2] = np.arctan2(np.sin(residual[:,2]), np.cos(residual[:,2]))
    cov = (residual.T * weights) @ residual
    return np.array([xy[0], xy[1], yaw]), cov


def covariance36(cov):
    result = [0.]*36
    for i, row in enumerate((0,1,5)):
        for j, col in enumerate((0,1,5)):
            result[row*6+col] = float(cov[i,j])
    return result


def checked_covariance(values, position_std=.25, heading_std=.30):
    matrix = np.asarray(values, dtype=float).reshape(6,6)
    planar = matrix[np.ix_([0,1,5], [0,1,5])]
    if not np.isfinite(matrix).all() or not np.allclose(planar, planar.T, atol=1e-8):
        raise ValueError('invalid pose covariance')
    if np.min(np.linalg.eigvalsh(planar)) < -1e-8:
        raise ValueError('non-PSD covariance')
    if np.max(np.linalg.eigvalsh(planar[:2,:2])) > position_std**2 or planar[2,2] > heading_std**2:
        raise ValueError('pose uncertainty too high')
    return planar


def checked_pose_timeout(value):
    """Explicit experimental pose-only tolerance; sensor gates stay unchanged."""
    value = float(value)
    if not math.isfinite(value) or not .20 <= value <= .30:
        raise ValueError('pose timeout must be finite and within 0.20..0.30 seconds')
    return value


class SourceFreshness:
    def __init__(self, timeout=.20, future_tolerance=.02):
        self.timeout, self.future_tolerance = timeout, future_tolerance
        self.stamp = None
        self.received = None

    def accept(self, stamp, ros_now, monotonic_now):
        if not all(math.isfinite(v) for v in (stamp,ros_now,monotonic_now)) or stamp <= 0.:
            return False
        if not -self.future_tolerance <= ros_now-stamp <= self.timeout:
            return False
        if self.stamp is not None and stamp <= self.stamp:
            return False
        self.stamp, self.received = stamp, monotonic_now
        return True

    def fresh(self, ros_now, monotonic_now):
        return (self.stamp is not None and
                0. <= monotonic_now-self.received <= self.timeout and
                -self.future_tolerance <= ros_now-self.stamp <= self.timeout)


class PurePursuitCore(OpenRouteMpcCore):
    def __init__(self, route, config=RouteMpcConfig(), lookahead=.4):
        super().__init__(route, config)
        if not math.isfinite(lookahead) or lookahead <= 0.:
            raise ValueError('invalid lookahead')
        self.lookahead = lookahead

    def step(self, x, y, yaw, measured_speed):
        self.pose = (x,y,yaw)
        return super().step(x,y,yaw,measured_speed)

    def _solve_control(self, initial, speeds, curvatures):
        target_s = min(float(self.s[-1]), float(self.s[self.progress_index])+self.lookahead)
        target_x = float(np.interp(target_s, self.s, self.route[:,0]))
        target_y = float(np.interp(target_s, self.s, self.route[:,1]))
        x,y,yaw = self.pose
        dx,dy = target_x-x, target_y-y
        local_x = math.cos(yaw)*dx+math.sin(yaw)*dy
        local_y = -math.sin(yaw)*dx+math.cos(yaw)*dy
        distance2 = dx*dx+dy*dy
        if local_x <= 0. or distance2 < 1e-8:
            self.solver_diagnostics = {'failure':'lookahead_not_ahead'}
            return None,None
        angle = math.atan2(2*self.config.wheelbase*local_y, distance2)
        angle = max(-self.config.max_steering, min(self.config.max_steering, angle))
        self.solver_diagnostics = {'controller':'pure_pursuit', 'failure':'',
                                   'lookahead_x':local_x, 'lookahead_y':local_y}
        return np.array([0.,angle]),None
