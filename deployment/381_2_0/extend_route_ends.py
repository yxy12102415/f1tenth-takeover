#!/usr/bin/env python3
"""Screen endpoint extensions on the saved occupancy map; never deploy them."""
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml
from PIL import Image
from scipy.integrate import solve_ivp
from scipy.ndimage import distance_transform_edt

D = Path(__file__).resolve().parent
WS = Path('/home/f1tenth/f1tenth_real_migration_ws_20260829')


def main():
    assets = json.loads((D/'assets/381_2_0.json').read_text())
    route = np.asarray(assets['route'])
    arc = np.r_[0., np.linalg.norm(np.diff(route[:, :2], axis=0), axis=1).cumsum()]
    curvature = np.gradient(np.unwrap(route[:, 2]), arc)
    meta = yaml.safe_load((WS/'maps/381_2_0.yaml').read_text())
    image = np.array(Image.open(meta['image']))
    resolution = float(meta['resolution'])
    origin = np.array(meta['origin'][:2])
    # Unknown cells are blocked, as are occupied cells.
    # This map_saver PGM contains exactly 0 (occupied), 205 (unknown), 254
    # (free). Explicitly exclude 205: its YAML free_thresh=0.25 would otherwise
    # classify that conventional unknown gray as free in a simple threshold.
    free = ((1. - image.astype(float)/255.) < meta['free_thresh']) & (image >= 250)
    padded = np.pad(free, 1, constant_values=False)
    field = distance_transform_edt(padded)[1:-1, 1:-1] * resolution

    def clearance(points):
        q = (points-origin)/resolution
        col = np.floor(q[..., 0]).astype(int)
        bottom_row = np.floor(q[..., 1]).astype(int)
        row = image.shape[0]-1-bottom_row
        valid = (col >= 0) & (col < image.shape[1]) & (row >= 0) & (row < image.shape[0])
        value = field[np.clip(row, 0, image.shape[0]-1), np.clip(col, 0, image.shape[1]-1)]
        # Lower bound to blocked cell area from arbitrary within-cell locations.
        center = origin + (np.stack([col, bottom_row], axis=-1)+.5)*resolution
        value = value - np.linalg.norm(points-center, axis=-1) - resolution/np.sqrt(2)
        return np.where(valid, np.maximum(0., value), 0.)

    offsets = np.array([(u, v) for u in np.linspace(-.29, .29, 25)
                        for v in np.linspace(-.155, .155, 15)])

    def evaluate(poses):
        c, s = np.cos(poses[:, 2]), np.sin(poses[:, 2])
        xx = poses[:, None, 0]+c[:, None]*offsets[:, 0]-s[:, None]*offsets[:, 1]
        yy = poses[:, None, 1]+s[:, None]*offsets[:, 0]+c[:, None]*offsets[:, 1]
        return clearance(poses[:, :2]), clearance(np.stack([xx, yy], axis=-1)).min(axis=1)

    results = {}
    extensions = []
    for name, index, sign in [('start', 0, -1.), ('end', -1, 1.)]:
        initial = route[index]
        k0 = curvature[index]
        # Continue the endpoint tangent and fade curvature continuously to zero
        # over 0.6 m. Beyond the blend the extension is straight.
        def derivative(distance, pose):
            u = min(1., distance/.6)
            k = k0*(1.-3.*u*u+2.*u*u*u)
            return sign*np.array([np.cos(pose[2]), np.sin(pose[2]), k])
        distances = np.linspace(0., 4., 401)
        poses = solve_ivp(derivative, (0., 4.), initial, t_eval=distances,
                          rtol=1e-10, atol=1e-11).y.T
        centers, bodies = evaluate(poses)
        valid = (centers >= .35) & (bodies >= .15)
        failures = np.flatnonzero(~valid)
        last = int(failures[0]-1) if len(failures) else len(valid)-1
        if last < 1:
            raise RuntimeError(f'{name} endpoint has no extension under the chosen margins')
        # Keep a further 0.20 m longitudinal reserve from the sampled limit.
        chosen = max(0, last-20)
        extensions.append(poses[:chosen+1])
        results[name] = {
            'sampled_extension_limit_m': float(distances[last]),
            'proposed_extension_m': float(distances[chosen]),
            'old_pose': initial.tolist(), 'proposed_pose': poses[chosen].tolist(),
            'minimum_center_clearance_lower_bound_m': float(centers[:chosen+1].min()),
            'minimum_sampled_body_clearance_lower_bound_m': float(bodies[:chosen+1].min()),
            'join_curvature_estimate_1pm': float(k0),
        }

    candidate = np.vstack([extensions[0][:0:-1], route, extensions[1][1:]])
    new_arc = np.r_[0., np.linalg.norm(np.diff(candidate[:, :2], axis=0), axis=1).cumsum()]
    k = np.gradient(np.unwrap(candidate[:, 2]), new_arc)
    centers, bodies = evaluate(candidate)
    report = {
        'map': '381_2_0', 'original_length_m': float(arc[-1]),
        'candidate_length_m': float(new_arc[-1]), 'extensions': results,
        'map_resolution_m': resolution, 'assumed_vehicle_length_width_m': [.58, .31],
        'dimensions_confirmed': False, 'extension_center_clearance_gate_m': .35,
        'extension_sampled_body_clearance_gate_m': .15,
        'whole_route_minimum_center_clearance_lower_bound_m': float(centers.min()),
        'whole_route_minimum_sampled_body_clearance_lower_bound_m': float(bodies.min()),
        'maximum_sampled_curvature_1pm': float(abs(k).max()),
        'physical_motion_ready': False, 'active_route_changed': False,
        'scope': 'Dense static map/footprint screen with unknown space blocked; no dynamic opponent, swept continuous-body certificate, speed profile, MPC preview or terminal stop acceptance.',
        'method': 'Retain original route. Integrate outward from both endpoint poses, fading estimated endpoint curvature to zero over 0.6 m. Sample extensions at 1 cm and body interior at <=2.5 cm; keep extra 0.20 m from sampled extension limit.',
    }
    (D/'reports/endpoint_extensions.json').write_text(json.dumps(report, indent=2)+'\n')
    with (D/'assets/381_2_0_extended_ends_candidate.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['x', 'y', 'yaw', 'distance_m', 'curvature_1pm'])
        writer.writerows(np.c_[candidate, new_arc, k])

    fig, ax = plt.subplots(figsize=(11, 9))
    extent = [origin[0], origin[0]+image.shape[1]*resolution,
              origin[1], origin[1]+image.shape[0]*resolution]
    ax.imshow(image, cmap='gray', origin='upper', extent=extent, vmin=0, vmax=255)
    ax.plot(route[:, 0], route[:, 1], color='#1674c0', lw=2.7, label=f'Original route: {arc[-1]:.2f} m')
    for path, name, color, text_offset in zip(extensions, ['start', 'end'], ['#00845a', '#ce5812'],
                                            [(-12, -55), (-65, 30)]):
        ax.plot(path[:, 0], path[:, 1], color=color, lw=3,
                label=f'{name.title()} extension: +{results[name]["proposed_extension_m"]:.2f} m')
        ax.scatter(*path[0, :2], facecolors='white', edgecolors=color, s=65, linewidths=2, zorder=5)
        ax.scatter(*path[-1, :2], color=color, marker='*', s=200, zorder=6)
        ax.annotate(f'New {name}\n({path[-1,0]:.2f}, {path[-1,1]:.2f})', path[-1, :2],
                    xytext=text_offset, textcoords='offset points', fontsize=10,
                    bbox=dict(facecolor='white', alpha=.85, edgecolor='none'),
                    arrowprops=dict(arrowstyle='-', color=color))
        # Show the assumed body at each proposed endpoint.
        x, y, yaw = path[-1]
        corners = np.array([[-.29,-.155],[.29,-.155],[.29,.155],[-.29,.155],[-.29,-.155]])
        rot = np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
        corners = corners@rot.T + [x,y]
        ax.plot(corners[:,0], corners[:,1], color=color, lw=1.2)
    ax.annotate('', xy=route[38,:2], xytext=route[32,:2],
                arrowprops=dict(arrowstyle='->', color='#1674c0', lw=2))
    ax.set(title=f'381 2.0 | Endpoint extension candidate: {new_arc[-1]:.2f} m',
           xlabel='Map X [m]', ylabel='Map Y [m]')
    ax.text(.02, .02, 'White: mapped free space | Black: occupied | Gray: unknown\n'
            'Circles: old endpoints | Stars: proposed endpoints\nStatic map screening only; not deployed',
            transform=ax.transAxes, fontsize=9,
            bbox=dict(facecolor='white', alpha=.9, edgecolor='none'))
    ax.legend(loc='upper left', fontsize=10)
    ax.set_aspect('equal')
    fig.tight_layout()
    fig.savefig(D/'reports/381_2_0_endpoint_extensions.png', dpi=170)
    fig.savefig(D/'reports/381_2_0_endpoint_extensions.pdf')
    plt.close(fig)
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
