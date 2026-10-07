"""Non-actuating actual ROS check before obstacle A; inherited fault injections.
The full lap with A and B is exercised by test_two_obstacles.py.
"""
from pathlib import Path
root=Path(__file__).resolve().parent
source=(root/'validate_cpp_ros.py').read_text()
source=source.replace("        enabled[0] = True\n        start = time.monotonic()", """        # Initialize lap progress at the real anchor, then place the synthetic
        # stationary fixture before A. This is not a motion simulation.
        pose[:] = np.loadtxt(profile, delimiter=',', skiprows=1)[70, :3]
        rays[:, 0] = pose[0]+.27*math.cos(pose[2])
        rays[:, 1] = pose[1]+.27*math.sin(pose[2])
        rays[:, 2] = pose[2]+angles
        caster.calc_range_many(rays, ranges)
        settle = time.monotonic()+1.0
        while time.monotonic()<settle:rclpy.spin_once(node,timeout_sec=.02)
        enabled[0] = True
        start = time.monotonic()""")
source=source.replace('cx, cy, half = 2.7497, -.2355, .0786','cx, cy, half = 6.022211142155291, 1.8324089029725348, .05')
source=source.replace('Stationary ground-truth pose, map-raycast 1081-beam scans','Stationary route pose at 3.49 m before A, map-raycast 1081-beam scans')
exec(compile(source,str(root/'validate_cpp_ros.py'),'exec'),dict(__file__=str(root/'validate_cpp_ros.py'),__name__='__main__'))
import sys,json
output=Path(sys.argv[sys.argv.index('--output')+1])
trace=json.loads(output.with_suffix('.trace.json').read_text())
active=[r for r in trace if r.get('valid') and r.get('speed_command',0)>0]
avoid=[r for r in active if r.get('state')=='AVOIDING' and r.get('confirmed_cluster_count',0)>=1]
assert len(avoid)>10,'ROS fixture did not exercise confirmed-obstacle avoidance'
report=json.loads(output.read_text());report['confirmed_avoidance_frames']=len(avoid);report['obstacle_center_xy']=[6.022211142155291,1.8324089029725348];output.write_text(json.dumps(report,indent=2)+'\n')
