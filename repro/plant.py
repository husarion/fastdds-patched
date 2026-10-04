#!/usr/bin/env python3
"""Plant a dead lock holder on a Fast DDS SHM port, then die by SIGKILL.

  plant.py sem <port>     sem_wait on sem.<dom>_port<port>_mutex (the port's named mutex)
  plant.py cvmutex <port> pthread_mutex_lock the PortNode's empty_cv_mutex
  plant.py processing <port> [<port>...]
                          mark every in-use listener slot of the ports as
                          processing a buffer (is_processing=1, is_waiting=0)
                          and exit: run it on a SIGSTOPped participant's own
                          ports, then SIGKILL it
  plant.py nowatch <port> [<port>...]
                          set the ports' last listener check time an hour
                          ahead, so no process's port watchdog checks them
  plant.py notok <port> [<port>...]
                          mark the ports not ok, as a watchdog does

Exactly what a participant SIGKILLed inside open_port_internal (sem) or inside
try_push/create_listener/wait_pop bookkeeping (cvmutex) leaves behind.
"""
import ctypes
import ctypes.util
import mmap
import os
import signal
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import locks  # noqa: E402

what = sys.argv[1]
shm = "/dev/shm"
if what in ("processing", "nowatch", "notok"):
    import struct
    import time
    for port in map(int, sys.argv[2:]):
        name = next(n for n in (f"fastrtps_port{port}", f"fastdds_port{port}") if os.path.exists(os.path.join(shm, n)))
        fd = os.open(os.path.join(shm, name), os.O_RDWR)
        mm = mmap.mmap(fd, 0)
        p, mo = locks.mutex_offset(bytes(mm), port)
        if p is None or mo is None:
            print(f"skipped {name}: port node not recognised", flush=True)
            continue
        if what == "nowatch":
            struct.pack_into("<q", mm, p, int(time.time() * 1000) + 3600 * 1000)
            print(f"planted nowatch on {name}", flush=True)
            continue
        if what == "notok":
            w = struct.unpack_from("<I", mm, p + 44)[0]
            struct.pack_into("<I", mm, p + 44, w & ~1)
            print(f"planted notok on {name}", flush=True)
            continue
        lst = mo + locks.MUTEX_SIZE
        n = 0
        for i in range(locks.LISTENERS):
            off = lst + i * locks.STATUS_SIZE
            if mm[off] & 1:
                mm[off] = (mm[off] | 4) & ~2 & 0xFF
                n += 1
        mm.flush()
        print(f"planted processing on {name}: {n} slot(s)", flush=True)
    sys.exit(0)
port = int(sys.argv[2])
name = next(n for n in (f"fastrtps_port{port}", f"fastdds_port{port}") if os.path.exists(os.path.join(shm, n)))
libc = ctypes.CDLL(ctypes.util.find_library("c"), use_errno=True)
if what == "sem":
    libc.sem_open.restype = ctypes.c_void_p
    s = libc.sem_open(f"/{name}_mutex".encode(), 0)
    assert s, "sem_open failed"
    libc.sem_wait(ctypes.c_void_p(s))
else:
    fd = os.open(os.path.join(shm, name), os.O_RDWR)
    mm = mmap.mmap(fd, 0)
    _p, mo = locks.mutex_offset(bytes(mm), port)
    addr = ctypes.addressof(ctypes.c_char.from_buffer(mm, mo))
    assert libc.pthread_mutex_lock(ctypes.c_void_p(addr)) == 0
print(f"planted {what} on {name}, dying", flush=True)
os.kill(os.getpid(), signal.SIGKILL)
