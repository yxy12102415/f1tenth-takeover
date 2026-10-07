"""Serialize this baseline's launchers; this is not a DDS security boundary."""
import fcntl
from pathlib import Path


def acquire():
    handle=(Path(__file__).resolve().parent/'session.lock').open('a')
    try:fcntl.flock(handle,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:
        handle.close()
        raise RuntimeError('REFUSED: another baseline session is already running')
    return handle
