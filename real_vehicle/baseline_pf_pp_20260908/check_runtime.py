"""Fail before ROS initialization if the tested dependency contract changes."""
import hashlib
import json
from pathlib import Path
import sys
import warnings

root = Path(__file__).resolve().parent


def check():
    with warnings.catch_warnings():
        warnings.simplefilter('error', RuntimeWarning)
        import numpy
        import range_libc
        import transforms3d
        import rclpy
        from baseline_pf_pp.safe_pf import SafeParticleFilter
    assert numpy.__version__ == '1.17.4', 'Untested NumPy version'
    assert str(Path(numpy.__file__)).startswith('/usr/lib/python3/dist-packages/'), 'Unexpected NumPy origin'
    assert str(Path(range_libc.__file__)).startswith(str(root/'dependencies/native_v1')+'/'), 'Global range_libc is forbidden'
    native_hash=hashlib.sha256(Path(range_libc.__file__).read_bytes()).hexdigest()
    assert native_hash=='0e6833e12f8b543f7eaf9243dfdf6fd1cdc84226e856310117ce4d5f981440ee', 'Native build differs from validated binary'
    assert str(Path(transforms3d.__file__)).startswith(str(root/'dependencies/python')+'/'), 'Unexpected transforms3d origin'
    assert not any('/f12026_ws/' in p or '/f1tenth-ws/' in p or '/.local/' in p for p in sys.path), 'Contaminated Python path'
    return {'python':sys.version.split()[0], 'numpy':numpy.__version__,
            'numpy_file':numpy.__file__, 'range_libc_file':range_libc.__file__,
            'range_libc_sha256':native_hash,
            'transforms3d_file':transforms3d.__file__, 'rclpy_file':rclpy.__file__,
            'sys_path':sys.path}


if __name__ == '__main__':
    print(json.dumps(check(), indent=2))
