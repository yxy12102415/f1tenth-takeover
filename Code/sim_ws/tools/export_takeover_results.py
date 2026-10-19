#!/usr/bin/env python3
"""Copy a named batch, including failed runs and compressed raw CSVs, for handoff."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import shutil


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('results_root', type=Path)
    parser.add_argument('run_prefix')
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    if args.destination.exists():
        raise SystemExit('Refusing to overwrite existing export: ' + str(args.destination))
    runs = sorted(p for p in args.results_root.glob('*/*')
                  if p.is_dir() and p.name.startswith(args.run_prefix))
    if not runs:
        raise SystemExit('No matching runs')
    args.destination.mkdir(parents=True)
    manifest = []
    for run in runs:
        dest = args.destination / run.parent.name / run.name
        dest.mkdir(parents=True)
        for source in sorted(run.iterdir()):
            if not source.is_file():
                continue
            if source.suffix == '.csv':
                target = dest / (source.name + '.gz')
                with source.open('rb') as src, target.open('wb') as out:
                    with gzip.GzipFile(filename='', mode='wb', fileobj=out, mtime=0) as zipped:
                        shutil.copyfileobj(src, zipped)
            elif source.suffix in ('.json', '.png', '.patch', '.log'):
                target = dest / source.name
                shutil.copyfile(source, target)
            else:
                continue
            manifest.append({'path': str(target.relative_to(args.destination)),
                             'sha256': hashlib.sha256(target.read_bytes()).hexdigest(),
                             'bytes': target.stat().st_size})
    for suffix in ('json', 'csv'):
        source = args.results_root / f'batch_{args.run_prefix}.{suffix}'
        if source.exists():
            shutil.copyfile(source, args.destination / source.name)
    (args.destination / 'MANIFEST.json').write_text(json.dumps({
        'run_prefix': args.run_prefix, 'run_count': len(runs),
        'source_root': str(args.results_root.resolve()),
        'note': 'All matching runs included, irrespective of outcome. CSV gzip is lossless. ROS subdirectory logs remain at source.',
        'files': manifest,
    }, indent=2) + '\n')
    print(f'Exported {len(runs)} runs to {args.destination}')


if __name__ == '__main__':
    main()
