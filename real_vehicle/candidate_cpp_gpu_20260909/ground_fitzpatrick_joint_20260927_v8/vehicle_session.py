#!/usr/bin/env python3
"""Fitzpatrick fixed-start single-lap PID / dynamic MPC session adapter."""
import argparse
import ctypes
import json
import os
from pathlib import Path
import subprocess
import sys

WS=Path(__file__).resolve().parents[2]
RELEASE=Path(__file__).resolve().parent

def cuda_ready():
    cuda=ctypes.CDLL('/usr/local/cuda/lib64/libcudart.so')
    cuda.cudaGetErrorString.restype=ctypes.c_char_p
    def checked(code):
        if code:
            raise RuntimeError('CUDA unavailable: '+cuda.cudaGetErrorString(code).decode()+
                '; check /dev/l3cache video-group access. Vehicle startup refused.')
    count=ctypes.c_int()
    checked(cuda.cudaGetDeviceCount(ctypes.byref(count)))
    if count.value<1:raise RuntimeError('No CUDA GPU; vehicle startup refused')
    pointer=ctypes.c_void_p()
    checked(cuda.cudaMalloc(ctypes.byref(pointer),ctypes.c_size_t(4096)))
    try:
        checked(cuda.cudaMemset(pointer,0,ctypes.c_size_t(4096)))
        checked(cuda.cudaDeviceSynchronize())
    finally:checked(cuda.cudaFree(pointer))

def desktop():
    candidates=[(os.environ.get('DISPLAY'),os.environ.get('XAUTHORITY'))]
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():continue
        try:
            if proc.stat().st_uid!=os.getuid():continue
            if (proc/'comm').read_text().strip() not in ('gnome-shell','gnome-session-b','xfce4-session'):continue
            env=dict(item.split('=',1) for item in (proc/'environ').read_bytes().decode(errors='replace').split('\0') if '=' in item)
            candidates.append((env.get('DISPLAY'),env.get('XAUTHORITY')))
        except (OSError,ValueError):continue
    tried=set()
    for display,authority in candidates:
        if not display or (display,authority) in tried:continue
        tried.add((display,authority));env=os.environ.copy();env['DISPLAY']=display
        if authority:env['XAUTHORITY']=authority
        try:
            if subprocess.run(['xdpyinfo'],env=env,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,timeout=2).returncode==0:
                return env
        except subprocess.TimeoutExpired:continue
    raise RuntimeError('No accessible graphical desktop. Log in to Ubuntu/NoMachine first.')

def evidence(env,motion):
    code='from motion_policy import verify_manifests,require_evidence; require_evidence(verify_manifests()["fingerprint"],'+repr(motion)+')'
    return subprocess.run([sys.executable,'-c',code],cwd=RELEASE,env=env,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode==0

def main():
    global RELEASE
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('tracker',choices=('pp','pid','dynamic_mpc'))
    parser.add_argument('mode',choices=('run','start','zero','check'))
    parser.add_argument('--baseline',action='store_true',help='Use the preserved steering-only MPC / original PID baseline')
    args=parser.parse_args()
    if args.tracker=='pp' or args.baseline:raise RuntimeError('This release supports PID and dynamic_mpc only')
    RELEASE=Path(__file__).resolve().parent
    cuda_ready();env=desktop();env['FRENET_TRACKER']=args.tracker
    if not Path('/dev/sensors/vesc').exists():raise RuntimeError('VESC serial device missing; restore USB/cdc_acm before starting.')
    print('GPU check passed; desktop '+env['DISPLAY']+'; C++ '+args.tracker+'; release '+RELEASE.name,flush=True)
    if args.mode=='check':return
    if args.mode=='run' and not evidence(env,False):
        subprocess.run(['bash','start.sh','--offline-self-test','--fault','window','--duration','90'],cwd=RELEASE,env=env,check=True)
    subprocess.run([str(WS/'ensure_ps4_controller.sh')],env=env,check=True)
    os.chdir(RELEASE)
    common=['bash',str(RELEASE/'start.sh'),'--ground-confirmed','--route-clear-confirmed','--cutoff-confirmed','--route-start-confirmed']
    zero=common+['--zero-only','--require-localization','--zero-rviz','--zero-joy','--zero-seconds','5','--duration','110']
    if args.mode=='zero':os.execvpe(zero[0],zero,env)
    if args.mode=='run':
        if not evidence(env,True):subprocess.run(zero,cwd=RELEASE,env=env,check=True)
        else:print('Reusing current-controller zero/localization evidence within 15 minutes.',flush=True)
    cuda_ready()
    start=common+['--allow-ground-motion','--duration','200']
    os.execvpe(start[0],start,env)

if __name__=='__main__':
    try:main()
    except (RuntimeError,OSError,subprocess.CalledProcessError) as error:
        print('REFUSED: '+str(error),file=sys.stderr);sys.exit(1)
