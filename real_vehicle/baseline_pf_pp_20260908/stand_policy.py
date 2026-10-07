"""Pure launch policy. No ground mode or implicit hardware connection."""
import math


def validate(args,domain,display,tty):
    if getattr(args,'route_start_confirmed',False):
        if args.offline_self_test or getattr(args,'zero_route_seed',False):
            raise ValueError('Confirmed route start requires a physical mode and cannot be an unverified seed')
        if args.zero_only and not getattr(args,'require_localization',False):
            raise ValueError('Confirmed route start requires localization checks')
    if getattr(args,'zero_route_seed',False) and (not args.zero_only or args.offline_self_test
            or args.allow_wheel_motion or not getattr(args,'require_localization',False)):
        raise ValueError('Route seed is an unverified initial guess for physical zero-only localization diagnostics only')
    if not 10.<=args.duration<=300. or not math.isfinite(args.duration):raise ValueError('duration must be 10..300 seconds')
    if args.offline_self_test:
        if args.stand_confirmed or args.allow_wheel_motion or args.zero_only:raise ValueError('Do not combine offline and physical modes')
        if domain!='83':raise ValueError('Offline domain must be 83')
    else:
        if not args.stand_confirmed:raise ValueError('Four wheels off the ground and cutoff confirmation required')
        if domain!='85':raise ValueError('Live stand domain must be 85')
        if args.allow_wheel_motion==args.zero_only:raise ValueError('Choose explicit zero-only OR allow-wheel-motion')
        if args.allow_wheel_motion and (not display or not tty):raise ValueError('Motion mode requires a desktop DISPLAY and interactive terminal')
