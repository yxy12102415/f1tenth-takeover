"""ctypes.CDLL releases the GIL during the bounded native iteration kernel.

No silent Python fallback: the frozen shared library is required at startup.
The state model, cost, warm start, constraints and stopping tolerance remain
those of the original tracker; this is not a different optimization algorithm.
"""
import ctypes
from pathlib import Path

import numpy as np

LIBRARY = Path(__file__).resolve().parent/'lib/libmpc_iteration.so'
_lib = ctypes.CDLL(str(LIBRARY))
_lib.mpc_iteration_abi.argtypes = []
_lib.mpc_iteration_abi.restype = ctypes.c_int
if _lib.mpc_iteration_abi() != 1:
    raise RuntimeError('mpc_native_abi_mismatch')
_array = np.ctypeslib.ndpointer(dtype=np.float64, flags='C_CONTIGUOUS')
_solve = _lib.mpc_projected_iterations
_solve.argtypes = [ctypes.c_int, ctypes.c_int, _array, _array, _array,
                   ctypes.c_double, ctypes.c_double, ctypes.c_double,
                   ctypes.c_double, ctypes.c_double, _array]
_solve.restype = ctypes.c_int


def projected_iterations(hessian, linear, warm_start, angle, steering_limit,
                         first_step, future_step, lipschitz, maximum_iterations):
    hessian = np.ascontiguousarray(hessian, dtype=np.float64)
    linear = np.ascontiguousarray(linear, dtype=np.float64)
    warm_start = np.ascontiguousarray(warm_start, dtype=np.float64)
    n = len(warm_start)
    if hessian.shape != (n, n) or linear.shape != (n,) or warm_start.shape != (n,):
        raise ValueError('mpc_native_shape')
    output = np.empty(n, dtype=np.float64)
    iterations = _solve(n, maximum_iterations, hessian, linear, warm_start,
                         angle, steering_limit, first_step, future_step,
                         lipschitz, output)
    if iterations < 0:
        raise ValueError('mpc_non_finite' if iterations == -2 else 'mpc_native_input')
    return output, iterations
