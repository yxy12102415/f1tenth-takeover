"""Non-actuating route/box rollout with a dynamic plant and servo lag.

Runs both actual VESC-style velocity feedback and ideal full-state feedback.
This is a model regression, not evidence of real tire identification or clearance.
"""
import json
import math
from pathlib import Path
import threading
import time
from types import SimpleNamespace, MethodType

import numpy as np

from avoidance_controller import (AvoidanceController, PLANNER_TUNING,
    LapProgress, load_speed_profile, open_endpoint_reached, endpoint_pose_error)
from return_lattice_planner import ClosedLoopLatticePlanner
from local_path_tracker import LocalPathTracker, model_parameters
from test_closed_lattice_controller import square_box_points
from test_dynamic_mpc import reference_continuous

ROOT = Path(__file__).resolve().parent
WORKSPACE = ROOT.parents[1]


def rollout(feedback, obstacle=True, initial=None, planner_class=ClosedLoopLatticePlanner):
    planner = planner_class(
        WORKSPACE/'routes/manual_route_20260923_172359_controller_ready.csv',
        WORKSPACE/'maps/381_2_0.yaml', static_echo_tolerance_m=.25, **PLANNER_TUNING)
    distance, profile = load_speed_profile(
        WORKSPACE/'routes/manual_route_20260923_172359_controller_ready_speed_profile_v2.csv',
        planner.route)
    node = SimpleNamespace(planner=planner, tracker=LocalPathTracker(),
        lap=LapProgress(planner.segment_m, closed_loop=False), complete=False,
        input_lock=threading.RLock(), tracker_lock=threading.RLock(),
        last_tracker_tick=None, last_angle=0., profile_distance=distance, profile_speed=profile,
        measured_speed=0., measured_lateral_speed=0., measured_yaw_rate=0.,
        lattice_state='FOLLOWING')
    node.reset_tracker = MethodType(AvoidanceController.reset_tracker, node)
    box = square_box_points((2.7497, -.2355), half_size=.0786) if obstacle else np.empty((0,2))
    x, y, heading = planner.route[0]
    speed = vy = rate = servo = command = 0.
    if initial is not None:
        x,y,heading,speed,command = initial
        servo = command
        rate = speed/.25*math.tan(command)
        progress = planner.nearest_progress(x,y)
        node.lap.previous_index = progress
        node.lap.distance_m = float(np.sum(planner.segment_m[:progress]))
    rows = []; failure = None; plan_times = []; selected = None
    for tick in range(500):
        if tick % 6 == 0:
            started = time.perf_counter()
            selected, _ = planner.plan(x, y, box, heading=heading,
                horizon_m=max(0., node.lap.route_length_m-node.lap.distance_m))
            plan_times.append(time.perf_counter()-started)
            node.lap.update(planner.progress_index)
            if open_endpoint_reached(False, node.lap.route_length_m, node.lap.distance_m,
                    *endpoint_pose_error(planner.route,x,y,heading)):
                node.complete = True; break
            if not selected.valid:
                failure = selected.reason; break
            node.lattice_state = planner.mode
        node.measured_speed = speed
        node.measured_lateral_speed = vy if feedback == 'full_state' else 0.
        node.measured_yaw_rate = rate if feedback == 'full_state' else speed/.25*math.tan(command)
        # Modest deterministic pose noise, as used in the earlier timing work.
        try:
            target, command = AvoidanceController.control(
                node, selected.path, x, y+.004*math.sin(tick*1.1),
                heading+.008*math.sin(tick*.8), 10.+tick*.05)
        except ValueError as error:
            failure = str(error); break
        if node.complete:
            break
        # Match the existing 1 m/s² command ramp and 9375-ERPM (1.25 m/s) driver cap.
        speed = min(target, speed+.05, 1.25)
        ac, bc, _ = reference_continuous(max(.5, speed), model_parameters())
        for _ in range(10):
            dt = .005
            servo += (command-servo)*dt/.10
            if speed < .5:
                vy = 0.; rate = speed/.25*math.tan(servo)
            else:
                velocity = np.array([vy, rate])
                velocity += dt*(ac[2:,2:]@velocity+bc[2:]*servo)
                vy, rate = velocity
            x += (speed*math.cos(heading)-vy*math.sin(heading))*dt
            y += (speed*math.sin(heading)+vy*math.cos(heading))*dt
            heading += rate*dt
        centers = np.array([x,y])+np.array([-.08,0.,.08])[:,None]*np.array([math.cos(heading),math.sin(heading)])
        clearance = float(np.min(np.linalg.norm(centers[:,None,:]-box[None,:,:],axis=2))) if len(box) else None
        rows.append(dict(t=tick*.05, x=float(x), y=float(y), heading=float(heading),
                         speed=float(speed), steering=float(command), clearance_to_obstacle_points_m=clearance,
                         progress_m=node.lap.distance_m, tracker=dict(node.tracker.last)))
    minimum = min((r['clearance_to_obstacle_points_m'] for r in rows if r['clearance_to_obstacle_points_m'] is not None), default=None)
    result = dict(feedback=feedback, obstacle=obstacle, complete=node.complete, failure=failure,
                  duration_s=len(rows)*.05, progress_m=node.lap.distance_m,
                  endpoint_distance_m=float(np.hypot(x-planner.route[-1,0],y-planner.route[-1,1])),
                  minimum_footprint_disc_center_to_obstacle_point_m=minimum,
                  maximum_cross_track_error_m=max(abs(r['tracker']['cross_track_error_m']) for r in rows),
                  maximum_planner_ms=max(plan_times)*1000.,
                  maximum_mpc_ms=max(r['tracker']['mpc_solve_ms'] for r in rows),
                  candidate_switches=planner.candidate_switch_count)
    result['passed'] = bool(node.complete and failure is None and (minimum is None or minimum>.15))
    return result, rows


def main():
    reports = []
    for mode, obstacle in [('vesc_estimate',False), ('vesc_estimate',True), ('full_state',True)]:
        report, rows = rollout(mode, obstacle)
        name = mode+('_box' if obstacle else '_clear')
        (ROOT/'run_logs'/('rollout_'+name+'.json')).write_text(json.dumps(dict(summary=report,trace=rows),indent=2)+'\n')
        print(json.dumps(report), flush=True); reports.append(report)
    (ROOT/'run_logs/dynamic_rollout_summary.json').write_text(json.dumps(reports,indent=2)+'\n')
    if not all(r['passed'] for r in reports):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
