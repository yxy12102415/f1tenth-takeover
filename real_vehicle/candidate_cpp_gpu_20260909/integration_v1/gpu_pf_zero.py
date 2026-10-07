"""GPU PF candidate entry point, offline domain only; refuses unsafe batch size."""
import os
from pathlib import Path
import sys
import numpy as np

ROOT=Path(__file__).resolve().parent.parent
if os.environ.get('ROS_DOMAIN_ID') not in ('83','85') or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
    raise RuntimeError('GPU zero-diagnostic candidate requires domain 83/85 localhost 1')
sys.path.insert(0,str(ROOT/'gpu_native_v1'))
import range_libc
if not range_libc.SHOULD_USE_CUDA or Path(range_libc.__file__).parent!=ROOT/'gpu_native_v1':
    raise RuntimeError('Wrong native extension or no compiled CUDA support')
import rclpy
from baseline_pf_pp.safe_pf import SafeParticleFilter

class Candidate(SafeParticleFilter):
    def __init__(self):
        super().__init__()
        if self.MAX_PARTICLES!=1000 or self.WHICH_RM!='rmgpu' or self.RANGELIB_VAR!=2 or self.ANGLE_STEP!=12:
            raise RuntimeError('Candidate profile must use 1000 particles, rmgpu, variant 2, step 12')
        # The ground release uses one 500 ms source/receipt freshness budget
        # throughout the localization, planning and command chain.  The
        # baseline PF class retains its historical 200 ms default, so set the
        # candidate clocks explicitly instead of silently inheriting it.
        self.scan_clock.timeout = .50
        self.odom_clock.timeout = .50
        if self.scan_clock.timeout != .50 or self.odom_clock.timeout != .50:
            raise RuntimeError('Candidate PF freshness budget must remain 0.50 seconds')

    def lidarCB(self,msg):
        count=len(msg.ranges[::self.ANGLE_STEP])
        # Upstream GPU multi-chunk path uses ceil and wrong last-chunk offsets.
        # Do NOT exercise it or silently fall back to CPU in this experiment.
        if count<1 or count*self.MAX_PARTICLES>262144:
            self.fault('gpu_batch_outside_reviewed_capacity');self.health();return
        super().lidarCB(msg)

def main():
    np.random.seed(20260909);rclpy.init();node=Candidate()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
