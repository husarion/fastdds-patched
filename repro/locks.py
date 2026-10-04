#!/usr/bin/env python3
"""Who holds a Fast DDS SHM port's two locks right now.

  listeners_status         the listener table right after that mutex: per slot,
                           byte 0 = is_in_use | is_waiting << 1 | is_processing << 2

  sem.<dom>_port<N>_mutex  the port's named mutex (a POSIX semaphore, glibc
                           new_sem: low 32 bits of the first 8 bytes = value,
                           1 free, 0 held)
  empty_cv_mutex           the process-shared pthread mutex inside the PortNode,
                           just before the listener table (__lock at +0,
                           __owner tid at +8)

For every port file (or --port N) prints one line: the semaphore value, the
mutex lock word and owner tid, and whether that tid is alive. With --selftest
it locks both itself and checks it reads what it set.

Usage: locks.py [--port N] [--shm /dev/shm] [--json] [--selftest]
"""
import argparse
import ctypes
import ctypes.util
import json
import mmap
import os
import platform
import struct
import sys

LISTENERS = 1024
STATUS_SIZE = 20
MUTEX_SIZE = 48 if platform.machine() in ("aarch64", "arm64") else 40
DOMAIN_NAMES = (b"fastrtps", b"fastdds")


def find_node(buf, port_id):
    want = struct.pack("<I", port_id)
    start = 0
    while True:
        k = buf.find(want, start)
        if k < 0:
            return None
        start = k + 1
        p = k - 20
        if p < 0 or p % 8 or p + 64 > len(buf):
            continue
        hc, pw, maxd = struct.unpack_from("<III", buf, p + 28)
        if hc == 0 or pw != hc // 3 or not 0 < maxd <= (1 << 20):
            continue
        return p


def mutex_offset(buf, port_id):
    p = find_node(buf, port_id)
    if p is None:
        return None, None
    doms = [d for d in (buf.find(n + b"\x00", p + 56) for n in DOMAIN_NAMES) if d > 0]
    if not doms:
        return p, None
    lst = min(doms) - LISTENERS * STATUS_SIZE
    return p, lst - MUTEX_SIZE


def tid_alive(tid):
    if tid <= 0:
        return None
    return os.path.exists(f"/proc/{tid}") or any(
        os.path.exists(f"/proc/{p}/task/{tid}") for p in os.listdir("/proc") if p.isdigit())


def read_port(shm, name, port_id):
    out = {"port": port_id, "name": name}
    sem = os.path.join(shm, f"sem.{name}_mutex")
    try:
        with open(sem, "rb") as f:
            b = f.read(8)
        out["sem_ino"] = os.stat(sem).st_ino
        out["sem_value"] = struct.unpack("<I", b[:4])[0]
        out["sem_waiters"] = struct.unpack("<I", b[4:8])[0]
    except OSError:
        out["sem_value"] = None
    path = os.path.join(shm, name)
    try:
        with open(path, "rb") as f:
            buf = f.read()
        out["seg_ino"] = os.stat(path).st_ino
    except OSError:
        return out
    p, mo = mutex_offset(buf, port_id)
    if mo is None:
        out["mutex"] = "unrecognised"
        return out
    lock, count, owner = struct.unpack_from("<iIi", buf, mo)
    lst = mo + MUTEX_SIZE
    # ListenerStatus: byte 0 bit 0 is_in_use, bit 1 is_waiting, bit 2 is_processing
    flags = [buf[lst + i * STATUS_SIZE] for i in range(LISTENERS)]
    out["listeners_in_use"] = sum(1 for f in flags if f & 1)
    out["listeners_processing"] = sum(1 for f in flags if f & 1 and f & 4)
    out["is_port_ok"] = bool(struct.unpack_from("<I", buf, p + 44)[0] & 1)
    out["mutex_lock"] = lock
    out["mutex_owner"] = owner
    out["mutex_owner_alive"] = tid_alive(owner)
    return out


def all_ports(shm, only):
    res = []
    for n in sorted(os.listdir(shm)):
        for dom in ("fastrtps", "fastdds"):
            pre = dom + "_port"
            if n.startswith(pre) and n[len(pre):].isdigit():
                pid = int(n[len(pre):])
                if only is None or pid == only:
                    res.append(read_port(shm, n, pid))
    return res


def selftest(shm):
    """Lock a port's empty_cv_mutex and its semaphore ourselves and read them back."""
    ports = all_ports(shm, None)
    if not ports:
        print("selftest: no port files"); return 1
    t = ports[0]
    name, port_id = t["name"], t["port"]
    libc = ctypes.CDLL(ctypes.util.find_library("c"), use_errno=True)
    fd = os.open(os.path.join(shm, name), os.O_RDWR)
    mm = mmap.mmap(fd, 0)
    buf = bytes(mm)
    p, mo = mutex_offset(buf, port_id)
    addr = ctypes.addressof(ctypes.c_char.from_buffer(mm, mo))
    r = libc.pthread_mutex_lock(ctypes.c_void_p(addr))
    tid = libc.gettid() if hasattr(libc, "gettid") else threading_tid()
    rd = read_port(shm, name, port_id)
    ok_mutex = rd.get("mutex_owner") == tid and rd.get("mutex_lock") != 0
    libc.pthread_mutex_unlock(ctypes.c_void_p(addr))
    rd2 = read_port(shm, name, port_id)
    ok_unlock = rd2.get("mutex_lock") == 0 and rd2.get("mutex_owner") == 0
    libc.sem_open.restype = ctypes.c_void_p
    s = libc.sem_open(f"/{name}_mutex".encode(), 0)
    ok_sem = None
    if s:
        before = read_port(shm, name, port_id).get("sem_value")
        libc.sem_wait(ctypes.c_void_p(s))
        held = read_port(shm, name, port_id).get("sem_value")
        libc.sem_post(ctypes.c_void_p(s))
        ok_sem = (before, held) == (1, 0)
    print(f"selftest {name}: lock={r} tid={tid} read_owner={rd.get('mutex_owner')} lock_word={rd.get('mutex_lock')} "
          f"mutex_read={'ok' if ok_mutex else 'BAD'} unlock_read={'ok' if ok_unlock else 'BAD'} sem_read={ok_sem}")
    return 0 if ok_mutex and ok_unlock and ok_sem is not False else 1


def threading_tid():
    return ctypes.CDLL(None).syscall(186)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int)
    ap.add_argument("--shm", default="/dev/shm")
    ap.add_argument("--json", action="store_true")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        sys.exit(selftest(a.shm))
    res = all_ports(a.shm, a.port)
    if a.json:
        print(json.dumps(res))
    else:
        for r in res:
            bad = (r.get("sem_value") == 0) or (r.get("mutex_lock") not in (0, None))
            print(("HELD " if bad else "free ") + json.dumps(r))
