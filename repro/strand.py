#!/usr/bin/env python3
"""What a process stranded on the discovery port sees, and what its threads do.

For each pid: every segment of port <port> it maps (unlinked ones included,
read through /proc/<pid>/mem, needs CAP_SYS_PTRACE) with its inode, whether
it is the port file, is_port_ok, the listener slots in use / waiting /
processing and the empty_cv_mutex lock word and owner; then every thread with
its name, state and wait channel. Also prints the port file's own reading.

Usage: strand.py <port> <pid> [<pid> ...]
"""
import os
import re
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import locks  # noqa: E402

port = int(sys.argv[1])
PAT = re.compile(r"^([0-9a-f]+)-([0-9a-f]+) \S+ \S+ \S+ (\d+)\s+/dev/shm/fast(?:rtps|dds)_port" + str(port) + r"( \(deleted\))?$")
file_ino = None
for n in (f"fastrtps_port{port}", f"fastdds_port{port}"):
    if os.path.exists(f"/dev/shm/{n}"):
        file_ino = os.stat(f"/dev/shm/{n}").st_ino
        r = locks.read_port("/dev/shm", n, port)
        print(f"FILE ino {file_ino} is_port_ok {int(bool(r.get('is_port_ok')))} listeners {r.get('listeners_in_use')} "
              f"processing {r.get('listeners_processing')} sem_value {r.get('sem_value')} sem_ino {r.get('sem_ino')}")


def read(buf):
    p, mo = locks.mutex_offset(buf, port)
    if mo is None:
        return None
    lock, _c, owner = struct.unpack_from("<iIi", buf, mo)
    lst = mo + locks.MUTEX_SIZE
    flags = [buf[lst + i * locks.STATUS_SIZE] for i in range(locks.LISTENERS)]
    return {"ok": int(bool(struct.unpack_from("<I", buf, p + 44)[0] & 1)),
            "in_use": sum(1 for f in flags if f & 1),
            "waiting": sum(1 for f in flags if f & 1 and f & 2),
            "processing": sum(1 for f in flags if f & 1 and f & 4),
            "lock": lock, "owner": owner}


for pid in sys.argv[2:]:
    try:
        cmd = open(f"/proc/{pid}/cmdline").read().replace("\0", " ")[:70]
        maps = open(f"/proc/{pid}/maps").read().splitlines()
    except OSError:
        print(f"PID {pid} gone")
        continue
    print(f"PID {pid} {cmd}")
    seen = set()
    with open(f"/proc/{pid}/mem", "rb", buffering=0) as mem:
        for line in maps:
            m = PAT.match(line)
            if not m or int(m[3]) in seen:
                continue
            ino = int(m[3])
            seen.add(ino)
            lo, hi = int(m[1], 16), int(m[2], 16)
            try:
                mem.seek(lo)
                r = read(mem.read(hi - lo))
            except OSError:
                r = None
            print(f"  SEG ino {ino}{' (deleted)' if m[4] else ''}{' FILE' if ino == file_ino else ''} {r}")
    for t in sorted(os.listdir(f"/proc/{pid}/task"), key=int):
        try:
            name = open(f"/proc/{pid}/task/{t}/comm").read().strip()
            state = open(f"/proc/{pid}/task/{t}/stat").read().rsplit(")", 1)[1].split()[0]
            wchan = open(f"/proc/{pid}/task/{t}/wchan").read().strip()
        except OSError:
            continue
        print(f"  TID {t} {name} {state} {wchan}")
