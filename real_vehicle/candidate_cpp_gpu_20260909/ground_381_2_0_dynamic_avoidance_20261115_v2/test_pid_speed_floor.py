"""Current 2m/s² commissioning-cap, floor and STOP regression."""
from pathlib import Path
import runpy
if __name__=='__main__':
    runpy.run_path(str(Path(__file__).resolve().with_name('test_generic_controls.py')),run_name='__main__')
