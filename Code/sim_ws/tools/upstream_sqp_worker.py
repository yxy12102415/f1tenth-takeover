#!/usr/bin/env python3
"""Local-only adapter for the pinned ForzaETH SQP core (NOT a GP reproduction).

No upstream implementation is vendored. Load only the named numerical methods
from an independently obtained, hash-verified checkout, without ROS 1 imports.
The phase boundary conditions replace upstream's raceline-to-raceline endpoint
constraints; all geometry is subsequently time-parameterized and checked in C++.
"""
import ast
import ctypes
import hashlib
import json
import os
from pathlib import Path
import sys
import time
import types
import warnings

import numpy as np
from scipy.interpolate import CubicSpline
from scipy.optimize import minimize, linprog, OptimizeResult, OptimizeWarning

COMMIT = "ddbfbb2312eccda7bb5d84469ec0980ddb07a5e9"
SOURCE_SHA256 = "6d0e83203c698b95528f2e472be299e6a2de3323e8d29dad443d1f704fcb0a0c"
METHODS = {
    "objective_function", "start_on_raceline_constraint", "obstacle_constraint",
    "consecutive_points_constraint", "turning_radius_constraint",
    "first_point_constraint", "combined_equality_constraints",
    "combined_inequality_constraints", "solve_sqp",
}


def objective_jac(d):
    """Derivative of upstream's scalar cost, independently checked below."""
    g = 20*np.asarray(d, dtype=float)
    second = 200*np.diff(d, n=2)
    g[:-2] += second
    g[1:-1] -= 2*second
    g[2:] += second
    first = 2000*(d[1]-d[0])
    g[0] -= first
    g[1] += first
    return g


def inequality_jac(solver, d):
    n = len(d)
    indices = solver.obs_downsampled_indices
    obstacles = np.zeros((len(indices), n))
    obstacles[np.arange(len(indices)), indices] = np.sign(d[indices]-solver.obs_downsampled_center_d)
    consecutive = np.zeros((max(0, len(indices)-1), n))
    second = np.diff(np.eye(n), n=2, axis=0)/(solver.down_sampled_delta_s**2)
    total = solver.global_traj_kappas-second @ d
    curvature = np.sign(total)[:, None]*second
    first = np.zeros((1, n))
    first[0, :2] = np.sign(d[1]-d[0])*np.array([1., -1.])
    jac = np.vstack((obstacles, consecutive, curvature, first))
    if hasattr(solver, 'dense_second'):
        dense_total = solver.dense_route_curvature-solver.dense_second @ d-solver.dense_bias
        jac = np.vstack((jac, np.sign(dense_total)[:, None]*solver.dense_second))
    return jac


def adapter_cost(solver, d):
    second = solver.dense_second @ d + solver.dense_bias
    return (solver.objective_function(d) + 1000*np.sum((d-solver.lateral_reference)**2)
            + 5*solver.spatial_length*np.mean(second**2))/1000


def adapter_cost_jac(solver, d):
    second = solver.dense_second @ d + solver.dense_bias
    return (objective_jac(d) + 2000*(d-solver.lateral_reference)
            + 10*solver.spatial_length/len(second)*solver.dense_second.T @ second)/1000


def corridor_feasibility(solver, bounds):
    """Necessary linear conditions, never a replacement for final validation.

    Upstream's zero-first-difference epsilon is enclosed by a small outer
    relaxation. Obstacle/noncrossing constraints are deliberately omitted:
    infeasibility of this SUPERSET can reject a corridor, feasibility cannot
    certify one. Keep the original SLSQP and all original residual checks.
    """
    n = len(bounds)
    eye = np.eye(n)
    second = np.diff(eye, n=2, axis=0)/solver.down_sampled_delta_s**2
    matrices, limits = [], []
    for matrix, offset, slack in (
        (second, solver.global_traj_kappas,
         2e-6/solver.down_sampled_delta_s**2+1e-4),
        (solver.dense_second, solver.dense_route_curvature-solver.dense_bias, 1e-4),
    ):
        matrices.extend((matrix, -matrix))
        limits.extend((solver.max_kappa+offset+slack, solver.max_kappa-offset+slack))
    matrices.extend(((eye[1]-eye[0])[None, :], (eye[0]-eye[1])[None, :]))
    limits.extend(([solver.down_sampled_delta_s+1e-4],)*2)
    # Include final acceptance tolerances in this necessary-condition test.
    matrices.extend((solver.equality_matrix, -solver.equality_matrix))
    limits.extend((solver.equality_target+1e-4, -solver.equality_target+1e-4))
    relaxed_bounds=np.asarray(bounds)+np.array([-1e-6, 1e-6])
    # HiGHS can write native diagnostics to stdout, which is our JSON protocol.
    # Redirect only the native call to stderr, restoring the fd even on error.
    saved = os.dup(1)
    try:
        os.dup2(2, 1)
        with warnings.catch_warnings():
            warnings.filterwarnings('ignore', category=OptimizeWarning,
                                    message='(Unrecognized options detected|Unknown solver options).*threads')
            return linprog(np.zeros(n), A_ub=np.vstack(matrices), b_ub=np.concatenate(limits),
                           bounds=relaxed_bounds, method='highs',
                           options={'time_limit': .015, 'threads': 1})
    finally:
        ctypes.CDLL(None).fflush(None)
        os.dup2(saved, 1)
        os.close(saved)


def load_core(source):
    data = Path(source).read_bytes()
    if hashlib.sha256(data).hexdigest() != SOURCE_SHA256:
        raise ValueError("upstream_source_hash_mismatch")
    tree = ast.parse(data, filename=str(source))
    node = next(n for n in tree.body if isinstance(n, ast.ClassDef)
                and n.name == "SQPAvoidanceNode")
    node.bases, node.decorator_list, node.keywords = [], [], []
    node.body = [n for n in node.body if isinstance(n, ast.FunctionDef) and n.name in METHODS]
    if {n.name for n in node.body} != METHODS:
        raise ValueError("upstream_numerical_interface_changed")
    def bounded_minimize(*args, **kwargs):
        # Adapter settings: upstream's ftol=0.1 can report success despite
        # appreciable constraint residuals. Tighten, never waive validation.
        solver = args[0].__self__
        phase = corridor_feasibility(solver, kwargs['bounds'])
        if phase.status != 0:
            return OptimizeResult(success=False, x=np.asarray(args[1]), nit=0,
                message='linear_corridor_infeasible' if phase.status == 2 else
                        'linear_corridor_check_failed:'+str(phase.status))
        if hasattr(solver, 'lateral_reference'):
            # The upstream knot-difference cost is unscaled by ds. Its effect
            # vanished with our denser knots, allowing centimetre-scale
            # curvature ripples. Penalize the actual C2 d'' in spatial units.
            args = (lambda d: adapter_cost(solver,d),) + args[1:]
            kwargs['jac'] = lambda d: adapter_cost_jac(solver,d)
        else:
            kwargs['jac'] = objective_jac
        kwargs['constraints'][0]['jac'] = lambda d: solver.equality_matrix
        kwargs['constraints'][1]['jac'] = lambda d: inequality_jac(solver, d)
        kwargs['options'] = dict(ftol=1e-8 if hasattr(solver, 'lateral_reference') else 1e-5,
                                 maxiter=40, disp=False)
        return minimize(*args, **kwargs)
    namespace = {"np": np, "minimize": bounded_minimize}
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(source), "exec"), namespace)
    return namespace["SQPAvoidanceNode"]


def solve(core, request):
    begin = time.monotonic()
    x = np.asarray(request["s"], dtype=float)
    initial = np.asarray(request["initial"], dtype=float)
    bounds = np.asarray(request["bounds"], dtype=float)
    n = len(x)
    if not 5 <= n <= 32 or initial.shape != (n,) or bounds.shape != (n, 2):
        raise ValueError("bad_shape")
    if not all(np.isfinite(a).all() for a in (x, initial, bounds)):
        raise ValueError("nonfinite_input")
    if np.any(bounds[:, 0] > bounds[:, 1]) or np.any(np.diff(x) <= 0):
        raise ValueError("empty_corridor")
    ds = x[1] - x[0]
    if not np.allclose(np.diff(x), ds):
        raise ValueError("nonuniform_knots")
    solver = core()
    solver.current_d = initial[0]
    if 'lateral_reference' in request:
        solver.lateral_reference = np.asarray(request['lateral_reference'], dtype=float)
        if solver.lateral_reference.shape != (n,) or not np.isfinite(solver.lateral_reference).all():
            raise ValueError('bad_lateral_reference')
    solver.down_sampled_delta_s = ds
    solver.spatial_length = x[-1]-x[0]
    solver.global_traj_kappas = np.asarray(request["curvature"], dtype=float)
    solver.max_kappa = float(request["max_curvature"])
    solver.obs_downsampled_indices = np.asarray(request["obstacle_indices"], dtype=int)
    solver.obs_downsampled_center_d = np.asarray(request["obstacle_d"], dtype=float)
    solver.obs_downsampled_min_dist = np.asarray(request["obstacle_clearance"], dtype=float)
    slope = float(request["initial_slope"])
    # The actual interpolant is affine in its knot values. Constrain its
    # endpoint curvature and dense second derivatives INSIDE optimization,
    # rather than correcting the output with an unoptimized steering blend.
    basis = CubicSpline(x, np.eye(n), axis=0, bc_type=((1, np.zeros(n)), (1, np.zeros(n))))
    bias = CubicSpline(x, np.zeros(n), bc_type=((1, slope), (1, 0.)))
    solver.equality_matrix = np.vstack((np.eye(n)[0], basis(x[0], 2), basis(x[-1], 2)))
    solver.equality_target = np.array([solver.current_d,
        float(request.get('initial_second_derivative', 0.))-bias(x[0], 2), -bias(x[-1], 2)])
    dense_x = np.linspace(x[0], x[-1], 101)
    solver.dense_second = basis(dense_x, 2)
    solver.dense_bias = bias(dense_x, 2)
    solver.dense_route_curvature = np.interp(dense_x, x[1:-1], solver.global_traj_kappas)
    original_inequality = solver.combined_inequality_constraints
    def dense_inequality(self, d):
        total = self.dense_route_curvature-self.dense_second @ d-self.dense_bias
        return np.concatenate((original_inequality(d), self.max_kappa-np.abs(total)))
    solver.combined_inequality_constraints = types.MethodType(dense_inequality, solver)
    # Local adapter, not an upstream bug fix: phase end holds the passing lane
    # instead of forcing an early return across a still-adjacent lead vehicle.
    # Unlike upstream's 0.02-abs(...) equality, start exactly at the measured d.
    def phase_boundary(self, d):
        return self.equality_matrix @ d-self.equality_target
    solver.start_on_raceline_constraint = types.MethodType(phase_boundary, solver)
    result = solver.solve_sqp(np.clip(initial, bounds[:, 0], bounds[:, 1]), bounds)
    equality = np.max(np.abs(solver.combined_equality_constraints(result.x)))
    inequality = np.min(solver.combined_inequality_constraints(result.x))
    bound_violation = max(np.max(bounds[:, 0] - result.x), np.max(result.x - bounds[:, 1]))
    valid = bool(result.success and np.isfinite(result.x).all() and
                 equality <= 1e-4 and inequality >= -1e-4 and bound_violation <= 1e-6)
    output = dict(ok=valid, message=str(result.message), iterations=int(result.nit),
                  solve_ms=1000*(time.monotonic()-begin),
                  equality_residual=float(equality), inequality_min=float(inequality),
                  bound_violation=float(bound_violation), upstream_commit=COMMIT)
    if valid:
        # C2 interpolation is an adapter addition. Do not assume its interior
        # remains feasible merely because knot constraints passed.
        spline = CubicSpline(x, result.x, bc_type=((1, slope), (1, 0.0)))
        output.update(s=x.tolist(), d=result.x.tolist(), coefficients=spline.c.T.tolist())
    return output


def main():
    core = load_core(sys.argv[1])
    print(json.dumps(dict(ready=True, upstream_commit=COMMIT, source_sha256=SOURCE_SHA256)), flush=True)
    traced = 0
    for line in sys.stdin:
        try:
            request = json.loads(line)
            output = solve(core, request)
            serialized = json.dumps(output, allow_nan=False)
            trace = os.environ.get('TAKEOVER_SQP_TRACE')
            if trace and traced < 100 and (not output['ok'] or output['solve_ms'] > 30):
                with open(trace, 'a') as handle:
                    handle.write(json.dumps(dict(request=request, result=output), allow_nan=False)+'\n')
                traced += 1
        except Exception as exc:
            serialized = json.dumps(dict(ok=False, message=str(exc)))
        print(serialized, flush=True)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
