"""One config source, with separate validated C++ caps for each tracker."""
from pathlib import Path
import json,math

def generate_profiles(root):
    root=Path(root);config=json.loads((root/'compact_config.json').read_text())
    for tracker,cap in config.get('generic_tracker_caps_mps',{}).items():
        if tracker not in ('pid','dynamic_mpc') or not math.isfinite(cap) or not .5<=cap<=config['generic_first_trial_effective_cap_mps']:
            raise ValueError('Invalid tracker commissioning cap')
        dest=root/'controller_profiles'/tracker;dest.mkdir(parents=True,exist_ok=True)
        derived=dict(config,generic_first_trial_effective_cap_mps=cap)
        (dest/'compact_config.json').write_text(json.dumps(derived,indent=2)+'\n')
        assets=dest/'runtime_assets.json'
        if not assets.exists():assets.symlink_to('../../runtime_assets.json')
        if assets.resolve()!=(root/'runtime_assets.json').resolve():raise ValueError('Controller route asset mismatch')

def assets_for_tracker(root,tracker):
    root=Path(root);config=json.loads((root/'compact_config.json').read_text())
    if tracker not in config.get('generic_tracker_caps_mps',{}):return root/'runtime_assets.json'
    cap=config['generic_tracker_caps_mps'][tracker]
    dest=root/'controller_profiles'/tracker
    expected=dict(config,generic_first_trial_effective_cap_mps=cap)
    if json.loads((dest/'compact_config.json').read_text())!=expected:raise ValueError('Tracker profile changed: regenerate and freeze before launch')
    if (dest/'runtime_assets.json').resolve()!=(root/'runtime_assets.json').resolve():raise ValueError('Tracker route changed')
    return dest/'runtime_assets.json'
