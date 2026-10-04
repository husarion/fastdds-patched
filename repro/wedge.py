#!/usr/bin/env python3
"""Which processes hold a shared-memory port's empty_cv_mutex with one of their own threads.

Reads every fastrtps_port/fastdds_port segment each pid maps, unlinked ones
included, through /proc/<pid>/mem (needs CAP_SYS_PTRACE), and prints one line
per segment whose empty_cv_mutex is locked: the port, whether the segment is
unlinked, is_port_ok, the listener slots in use and processing, the owner tid,
and whether that tid is a thread of the same process (with its name; a
mutex records its owner as the locker's own pid namespace numbers it, so a
containerised process's threads are matched by the last NSpid field). A
sender stuck in Port::recover_blocked_processing() holds the mutex of the
dead participant's port with its own thread.

Usage: wedge.py <pid> [<pid> ...]   exit 0; "SELF <pid> ..." lines mark a
                                    process that holds a port mutex itself
"""
import os
import re
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import locks  # noqa: E402

PAT = re.compile(r"^([0-9a-f]+)-([0-9a-f]+) \S+ \S+ \S+ (\d+)\s+/dev/shm/fast(?:rtps|dds)_port(\d+)( \(deleted\))?$")


def thread_name(pid, tid):
    try:
        with open(f"/proc/{pid}/task/{tid}/comm") as f:
            return f.read().strip()
    except OSError:
        return "?"


def ns_tids(pid):
    """{tid as the process's own pid namespace numbers it: host tid}"""
    out = {}
    try:
        tasks = os.listdir(f"/proc/{pid}/task")
    except OSError:
        return out
    for t in tasks:
        out[t] = t
        try:
            for line in open(f"/proc/{pid}/task/{t}/status"):
                if line.startswith("NSpid:"):
                    out[line.split()[-1]] = t
        except OSError:
            pass
    return out


def scan(pid):
    try:
        maps = open(f"/proc/{pid}/maps").read().splitlines()
    except OSError:
        return
    tasks = ns_tids(pid)
    seen = set()
    with open(f"/proc/{pid}/mem", "rb", buffering=0) as mem:
        for line in maps:
            m = PAT.match(line)
            if not m:
                continue
            lo, hi, ino, port, deleted = int(m[1], 16), int(m[2], 16), int(m[3]), int(m[4]), bool(m[5])
            if (ino, port) in seen:
                continue
            seen.add((ino, port))
            try:
                mem.seek(lo)
                buf = mem.read(hi - lo)
            except OSError:
                continue
            p, mo = locks.mutex_offset(buf, port)
            if mo is None:
                continue
            lock, _count, owner = struct.unpack_from("<iIi", buf, mo)
            if lock == 0:
                continue
            lst = mo + locks.MUTEX_SIZE
            flags = [buf[lst + i * locks.STATUS_SIZE] for i in range(locks.LISTENERS)]
            in_use = sum(1 for f in flags if f & 1)
            processing = sum(1 for f in flags if f & 1 and f & 4)
            ok = bool(struct.unpack_from("<I", buf, p + 44)[0] & 1)
            self_owned = str(owner) in tasks
            tag = "SELF" if self_owned else "HELD"
            who = (f"own thread {owner} (host tid {tasks[str(owner)]}, {thread_name(pid, tasks[str(owner)])})"
                   if self_owned else f"tid {owner} (not this process)")
            print(f"{tag} {pid} port {port}{' (deleted)' if deleted else ''} ino {ino} is_port_ok {int(ok)} "
                  f"listeners {in_use} processing {processing} mutex lock {lock} owner {who}", flush=True)


for a in sys.argv[1:]:
    scan(a)
