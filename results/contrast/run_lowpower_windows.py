"""External Windows launcher; does not change the simulation source or parameters."""
import ctypes
import json
import os
from pathlib import Path
import runpy
import sys

for name in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'VECLIB_MAXIMUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[name] = '1'
if os.name == 'nt':
    kernel = ctypes.WinDLL('kernel32', use_last_error=True)
    kernel.GetCurrentProcess.restype = ctypes.c_void_p
    kernel.SetPriorityClass.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
    kernel.GetPriorityClass.argtypes = [ctypes.c_void_p]
    handle = kernel.GetCurrentProcess()
    if not kernel.SetPriorityClass(handle, 0x40):
        raise ctypes.WinError(ctypes.get_last_error())
    assert kernel.GetPriorityClass(handle) == 0x40
    print('[LOW POWER] Windows IDLE_PRIORITY_CLASS; BLAS thread limits=1', flush=True)
else:
    os.nice(19)
sys.path.insert(0, str(Path.cwd()))
args = sys.argv[1:]
if args[0] == '-m':
    sys.argv = args[1:]
    runpy.run_module(args[1], run_name='__main__', alter_sys=True)
else:
    sys.argv = args
    runpy.run_path(args[0], run_name='__main__')
