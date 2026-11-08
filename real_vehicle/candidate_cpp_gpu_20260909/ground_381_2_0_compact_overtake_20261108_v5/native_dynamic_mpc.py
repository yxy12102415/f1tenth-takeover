"""Native four-state dynamic bicycle MPC; local-path geometry stays in Python."""
import ctypes

import numpy as np

from native_mpc import _lib, _array

_lib.dynamic_mpc_abi.argtypes = []
_lib.dynamic_mpc_abi.restype = ctypes.c_int
if _lib.dynamic_mpc_abi() != 1:
    raise RuntimeError('dynamic_mpc_abi_mismatch')
_solve = _lib.dynamic_mpc_solve
_solve.argtypes = [_array]*6 + [ctypes.c_double]*5 + [ctypes.c_int, _array, _array]
_solve.restype = ctypes.c_int
_discrete = _lib.dynamic_discrete_model
_discrete.argtypes = [ctypes.c_double, ctypes.c_double]+[_array]*4
_discrete.restype = ctypes.c_int


def vector(values, length):
    result = np.ascontiguousarray(values, dtype=np.float64)
    if result.shape != (length,) or not np.isfinite(result).all():
        raise ValueError('mpc_non_finite_or_shape')
    return result


def discrete_model(speed, dt, model):
    a = np.empty((4, 4)); b = np.empty(4); c = np.empty(4)
    if _discrete(speed, dt, vector(model, 6), a, b, c) != 0:
        raise ValueError('dynamic_discretization_singular')
    return a, b, c


def solve(initial, speeds, curvatures, model, weights, previous, angle,
          steering_limit, first_step, prediction_step, dt, iterations):
    output = np.empty(16); maxima = np.empty(4)
    result = _solve(vector(initial, 4), vector(speeds, 16), vector(curvatures, 16),
                    vector(model, 6), vector(weights, 10), vector(previous, 16),
                    angle, steering_limit, first_step, prediction_step, dt,
                    iterations, output, maxima)
    if result < 0:
        raise ValueError('dynamic_discretization_singular' if result == -2
                         else 'mpc_non_finite')
    return output, maxima, result
