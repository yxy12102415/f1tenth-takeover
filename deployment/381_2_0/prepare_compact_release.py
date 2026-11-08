#!/usr/bin/env python3
"""Create the independent, physically disabled compact overtaking candidate.

Does not build, freeze, launch ROS, or touch an existing destination.
"""
import argparse,hashlib,json,shutil,subprocess
from pathlib import Path
D=Path(__file__).resolve().parent
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--workspace',type=Path,default=Path('/home/f1tenth/f1tenth_real_migration_ws_20260829'));ap.add_argument('--name',default='ground_381_2_0_compact_overtake_20261107_v1');args=ap.parse_args()
    identity=json.loads((D/'assets/compact_base_identity.json').read_text())
    root=args.workspace/'candidate_cpp_gpu_20260909';src=root/identity['source_release'];dst=root/args.name
    if dst.exists():raise RuntimeError('destination already exists; refused')
    if dst.parent!=root or dst.name!=args.name:raise RuntimeError('destination must be one directory name')
    if hashlib.sha256((src/'src/cpp_frenet/node.cpp').read_bytes()).hexdigest()!=identity['base_node_sha256']:raise RuntimeError('source release changed; review required')
    shutil.copytree(src,dst,ignore=shutil.ignore_patterns('build_cpp','log_cpp','run_logs','__pycache__','runtime_manifest.sha256'))
    (dst/'run_logs').mkdir()
    subprocess.run(['patch','--batch','-p1','-i',str(D/'compact_node.patch')],cwd=dst,check=True)
    shutil.copyfile(D/'compact_overtake.hpp',dst/'src/cpp_frenet/compact_overtake.hpp')
    shutil.copyfile(D/'assets/compact_candidate.json',dst/'compact_config.json')
    # Future release evidence must include the experiment's config as well as
    # the C++ sources already included by the base release's glob.
    freeze=dst/'freeze.py';text=freeze.read_text();text=text.replace("if __name__=='__main__':","FILES += ['compact_config.json']\nif __name__=='__main__':");freeze.write_text(text)
    print(str(dst));print('Physical domain 85 remains refused; no vehicle was started.')
if __name__=='__main__':main()
