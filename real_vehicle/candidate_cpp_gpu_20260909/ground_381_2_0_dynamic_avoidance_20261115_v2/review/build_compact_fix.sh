#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
mkdir -p reports/compact_fix
python3 - <<'PY'
from pathlib import Path
import json,hashlib
r=Path('.');v={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted((r/'src/cpp_frenet').glob('*')) if p.is_file()};Path('reports/compact_fix/source_before.json').write_text(json.dumps(v,indent=2))
PY
if [[ ${REUSE_VERIFIED_NATIVE_PROBE:-0} == 1 ]]; then
  python3 - <<'CHECK'
from pathlib import Path
import subprocess
binary=Path('bin/generic_probe')
# Ask the compiler for the probe's actual local dependencies. Controller-only
# changes rebuild node.cpp below without recompiling an unaffected probe.
raw=subprocess.check_output(['g++','-std=c++14','-O2','-ffp-contract=off','-I/usr/include/eigen3','-MM','src/cpp_frenet/generic_probe.cpp'],text=True)
dependencies=[Path(n) for n in raw.replace(chr(92),' ').split() if n.startswith('src/cpp_frenet/')]
assert dependencies and binary.exists() and binary.stat().st_mtime>=max(p.stat().st_mtime for p in dependencies)
CHECK
else
g++ -std=c++14 -O2 -ffp-contract=off -I/usr/include/eigen3 src/cpp_frenet/generic_probe.cpp -ljsoncpp -o bin/generic_probe
fi
CMAKE_BUILD_PARALLEL_LEVEL=2 bash ../offline_env.sh colcon --log-base log_cpp build --base-paths src/cpp_frenet --build-base build_cpp --install-base install_cpp --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
python3 - <<'PY'
from pathlib import Path
import json,hashlib
r=Path('.');v={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted((r/'src/cpp_frenet').glob('*')) if p.is_file()};assert v==json.loads(Path('reports/compact_fix/source_before.json').read_text()),'source changed during build'
names=['cpp_frenet_planner','cpp_frenet_controller'];b={n:hashlib.sha256((r/'install_cpp/cpp_frenet/lib/cpp_frenet'/n).read_bytes()).hexdigest() for n in names};p=dict(source_unchanged_during_build=True,source_sha256=v,binaries=b,build_log=str(r.resolve()/'reports/compact_fix/build.log'),physical_motion=False);Path('build_provenance.json').write_text(json.dumps(p,indent=2)+'\n');Path('build_source_snapshot.json').write_text(json.dumps(v,indent=2)+'\n')
PY
python3 review/test_compact_fix.py
python3 test_dynamic_memory.py
printf '0\n' > reports/compact_fix/build.exit
