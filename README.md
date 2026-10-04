# fastdds-patched

Fast DDS as the ROS 2 apt repository ships it, rebuilt from the same source package with three shared-memory transport fixes that upstream does not have yet. Each package is a drop-in replacement for `ros-<distro>-fastrtps` (Humble, Jazzy) or `ros-<distro>-fastdds` (Lyrical) at the exact version our images install, with `+husarion<N>` appended to the version: `+husarion1` in v1 (fix 1), `+husarion4` in v2 (fixes 1 to 3; `+husarion2` and `+husarion3` were unreleased test builds).

All three bugs hit a host whose ROS processes share one shared-memory world and one of them dies uncleanly (SIGKILL, OOM, a crash). Each fix is inside a private header: the library's ABI and the shared-memory layout do not change, so patched and unpatched processes can share a world (each fix protects the processes that carry it).

## 1. A regenerated listener keeps its old slot (the closed world, v1)

When a process on a host notices that another Fast DDS participant died, it regenerates the shared-memory discovery port. `SharedMemManager::Listener::regenerate_port()` builds a new listener, which registers on the new port and stores its new slot number, then move-assigns it into the old one. The move assignment does not copy `listener_index_`, so every surviving process keeps writing its liveness into its old slot on the new port.

A participant that joins later and lands on such a slot can then die without anyone noticing, because a survivor keeps the slot's counter moving. Its registration is never removed, the port's 512-cell ring fills within a minute or two of normal traffic, and every later announcement on the port is dropped. Processes that were already running keep talking. Anything that starts afterwards sees only its own topics.

It takes two unclean deaths (SIGKILL, OOM, crash) after the last full reset of the host's ROS processes: the first arms the world, the second closes it. Every Fast DDS release since 2.0.0 has it, including 2.6.x (Humble), 2.14.x (Jazzy), 3.0 to 3.6 (Lyrical) and master as of 2026-10-03.

The fix is one line, in `patches/<upstream version>/`:

```diff
             global_port_ = other.global_port_;
             other.global_port_.reset();
+            listener_index_ = other.listener_index_;
             shared_mem_manager_ = other.shared_mem_manager_;
```

## 2. Several processes reset a dead owner's port mutex at once (the split world, v2)

Every open, regeneration and removal of a shared-memory port runs under the port's named mutex, a POSIX semaphore. A semaphore is not released when its owner dies, and Fast DDS's recovery is a 2 s timeout after which the waiter removes the name, creates a fresh semaphore and locks it. When a participant dies holding it (a process killed while it opens a port) and the port then needs regenerating, every survivor waits on it at once, times out within the same second and resets it for itself; several processes then open, remove and create the same port, and the discovery port ends as several segments, each still marked ok. A process left on an unlinked segment never hears anyone who joins later, and nothing heals it until the processes restart.

`0002` takes the port mutex through new `*_robust_named_mutex()` functions: every holder first holds a kernel `flock` on `<domain>_port<N>_mutex_holder` in the lock directory. The kernel releases a dead holder's flock, so the one process that then times out on the dead semaphore is the only one resetting it. The file is never removed, and it is opened without `O_CREAT` when it exists: in a sticky world-writable directory the kernel refuses `O_CREAT` on another user's file (`fs.protected_regular`), and robot worlds mix users (a root driver, a uid-10001 bridge).

## 3. A sender spins forever recovering a dead listener's buffers (the wedged sender, v2)

When a send finds a port marked not ok, `SharedMemManager::Port::try_push()` regenerates it, and for a zombie port (its listener's process is dead) it first runs `recover_blocked_processing()`, which calls `SharedMemGlobal::Port::get_and_remove_blocked_processing()` until no listener slot is marked as processing a buffer. That function holds the port's `empty_cv_mutex` and calls `listener_processing_stop()`, which locks the same non-recursive mutex again. The inner lock times out after 1 s, its exception is swallowed, the slot stays marked, the function returns true and the loop calls it again, forever.

A participant killed between taking a message and finishing it leaves exactly such a slot. The next process whose send to that port sees the port marked not ok (a port watchdog marks it at any moment, so a send under way is enough) spins in that loop on the sending thread, holding the participant's send lock: nothing it publishes reaches anyone, its discovery thread blocks as soon as it needs that lock (a new participant appears), and it stays on the old discovery segment when the other processes regenerate it. The process looks alive and never recovers. Only the first process that tests the dead port for a zombie can take this path (the test removes the port's lock file), so it strikes one process at a time.

`0003` clears the flag under the lock `get_and_remove_blocked_processing()` already holds, and bounds the loop in `recover_blocked_processing()` to one pass over the listener table:

```diff
                         buffer_descriptor = node_->listeners_status[i].descriptor;
-                        listener_processing_stop(i);
+                        node_->listeners_status[i].is_processing = false;
                         return true;
```

Present in 2.6.12, 2.14.6, 3.6.2 and master as of 2026-10-04.

## Evidence

`repro/closed_world.sh` on amd64, 2026-10-03, stock package against the patched one:

| Release | Package | Second death | Verdict |
|---|---|---|---|
| Humble | ros-humble-fastrtps 2.6.12 | not noticed | closed 100 s after the load |
| Humble | 2.6.12 `+husarion1` | noticed | open after 600 s |
| Jazzy | ros-jazzy-fastrtps 2.14.6 | not noticed | closed 134 s after the load |
| Jazzy | 2.14.6 `+husarion1` | noticed | open after 600 s |
| Lyrical | ros-lyrical-fastdds 3.6.2 | not noticed | closed 135 s after the load |
| Lyrical | 3.6.2 `+husarion1` | noticed | open after 600 s |

On a Lynx (UGV OS, arm64), the stock world closed 11 of 11 times under two SIGKILLs of one participant followed by a graceful restart.

v2, 2026-10-03 and 2026-10-04. `repro/split_world.sh` (fix 2: eighteen participants, a planted dead owner of the discovery port's semaphore, one participant SIGKILLed so every survivor regenerates the port; two publishers and a newcomer run as uid 10001; SPLIT when a process is left on an unlinked segment, BROKEN when a newcomer misses a publisher without one) and `repro/blocked_sender.sh` (fix 3: three publishers and a subscriber; the subscriber dies with its listeners marked processing, and gdb marks its ports not ok while the first publisher is inside a send to one of them; WEDGED when the restarted subscriber never hears a live publisher, and `wedge.py` shows that publisher holding the dead port's mutex with its own thread):

| Release | Architecture | Package | split_world.sh | blocked_sender.sh |
|---|---|---|---|---|
| Humble | amd64 | stock 2.6.12 | SPLIT 7 of 24 (runs of 8: 2, 5, 0) | WEDGED 6 of 6 |
| Humble | amd64 | `+husarion4` | WHOLE 0 of 16 | RECOVERED 0 of 6 |
| Jazzy | amd64 | stock 2.14.6 | SPLIT 6 of 24 (2, 1, 3; 4 of 8 on 2026-10-03) | WEDGED 4 of 4 |
| Jazzy | amd64 | `+husarion3` (fixes 1, 2) | WHOLE 0 of 8 | WEDGED 4 of 4 |
| Jazzy | amd64 | `+husarion4` (v2) | WHOLE 0 of 24 (a first run, beside a package build, read one uid-10001 newcomer at 9 of 10: BROKEN 1 of 8) | RECOVERED 0 of 6 |
| Lyrical | amd64 | stock 3.6.2 | SPLIT 6 of 24 (3, 1, 2) | WEDGED 6 of 6 |
| Lyrical | amd64 | `+husarion4` | WHOLE 0 of 16 | RECOVERED 0 of 6 |
| Humble | arm64 | stock | SPLIT 13 of 24 (4, 4, 5) | WEDGED 6 of 6 |
| Humble | arm64 | `+husarion4` | WHOLE 0 of 24 | RECOVERED 0 of 6 |
| Jazzy | arm64 | stock | SPLIT 6 of 24 (1, 2, 3) | WEDGED 6 of 6 |
| Jazzy | arm64 | `+husarion4` | WHOLE 0 of 24 | RECOVERED 0 of 6 |
| Lyrical | arm64 | stock | SPLIT 9 of 24 (3, 3, 3) | WEDGED 6 of 6 |
| Lyrical | arm64 | `+husarion4` | WHOLE 0 of 24 | RECOVERED 0 of 6 |

The split is a race: a stock round splits 6 to 13 times in 24 per cell (amd64 on the lab NUC, arm64 on a 10-core VM, 2026-10-04), and one 8-round stock run on Humble amd64 read WHOLE. A fixed 8-round stock run would therefore turn the release workflow red now and then on a step that says nothing about the fix. The stock run of `split_world.sh` stops at its first split and allows up to 48 rounds: at the lowest measured rate (6 of 24) it misses with a probability below 1e-5, and below 1 % even at 0.10, the 95 % lower bound of that rate. With the change, stock runs on amd64 split at round 4 (Humble), 1 (Jazzy) and 10 (Lyrical). The patched run stays strict: 8 rounds, no split and no broken round. The rates on GitHub's runners are not measured; `CASES` sets the round count of either run. `blocked_sender.sh` needs no such margin: the stock package wedged in all 34 of its rounds across the six cells (and in the 4 of `+husarion3`), and one wedged round of six is enough.

The controls of `blocked_sender.sh` recover on the stock package: the same in-send flip without the processing mark (`LEVER=flip`, 0 of 4) and a plain kill (`LEVER=none`, 0 of 4). `closed_world.sh` keeps reading OPEN on `+husarion4` (Humble, Jazzy, Lyrical amd64).

On the Lynx, every process of the robot's shared-memory world was hotpatched with each package. One process was SIGKILLed 3 s into its start, after being stopped and having its own ports' listener slots marked as processing (what a process killed mid-message leaves behind); the rest was left to the robot's timing. With `+husarion3` (fixes 1 and 2), a process was left wedged and stranded in 4 of 11 rounds. Each time, its event thread was waiting on the dead port's mutex, which it held itself. With `+husarion4`, this happened in 0 of 23 rounds. The plain kill without the mark split 3 of 10 rounds with `+husarion3` on 2026-10-03 and 0 of 10 on 2026-10-04.

## Every process needs it

A single unpatched process in the shared-memory world keeps writing into its stale slot (fix 1), resets a dead owner's port mutex without the file lock (fix 2) or can spin in the recovery loop (fix 3), so the world stays exposed. Install the package in every image whose processes share the robot's `/dev/shm` world: the driver, rosbridge, cameras, the airlock halves and anything else, including the `ros2` CLI that runs inside those containers.

## Using the packages

Each release `v<N>` carries one package per distro and architecture under a stable name, `<package>-<distro>-<arch>.deb` (`fastrtps-jazzy-arm64.deb`, `fastrtps-humble-amd64.deb`, `fastdds-lyrical-amd64.deb`), plus `SHA256SUMS` and `VERSIONS` (the full package version of each file). An image pins the release and the checksum, so a rebuilt release can never slip in unnoticed:

```dockerfile
# The Fast DDS shared-memory fixes (husarion/fastdds-patched). Every image whose
# processes share the robot's /dev/shm ROS world needs it. The last three
# checks prove it at build time: the patched package is installed, its files
# are intact, and no other copy of the library exists for a process to load.
ARG FASTDDS_PATCHED=v2
ARG FASTDDS_SHA256_AMD64=<sha256 of fastrtps-jazzy-amd64.deb from SHA256SUMS>
ARG FASTDDS_SHA256_ARM64=<sha256 of fastrtps-jazzy-arm64.deb from SHA256SUMS>
RUN set -eu; arch=$(dpkg --print-architecture); \
    case "$arch" in amd64) sum=$FASTDDS_SHA256_AMD64 ;; arm64) sum=$FASTDDS_SHA256_ARM64 ;; *) exit 1 ;; esac; \
    curl -fsSL -o /tmp/fastdds.deb \
      "https://github.com/husarion/fastdds-patched/releases/download/${FASTDDS_PATCHED}/fastrtps-${ROS_DISTRO}-${arch}.deb"; \
    echo "${sum}  /tmp/fastdds.deb" | sha256sum -c -; \
    apt-get update && apt-get install -y --no-install-recommends /tmp/fastdds.deb; \
    pkg=$(dpkg-deb -f /tmp/fastdds.deb Package); apt-mark hold "$pkg"; rm -f /tmp/fastdds.deb; \
    dpkg-query -W -f='${Version}' "$pkg" | grep -q '+husarion' ; \
    dpkg --verify "$pkg"; \
    test "$(find / -xdev \( -name 'libfastrtps.so*' -o -name 'libfastdds.so*' \) -type f -not -path "/opt/ros/${ROS_DISTRO}/lib/*" | wc -l)" = 0; \
    rm -rf /var/lib/apt/lists/*
```

Lyrical's package is `fastdds` (`fastdds-lyrical-<arch>.deb`). `apt-get install` takes no extra packages: the patched package declares exactly the stock package's dependencies, which every image with the stock package already has. If the image already carries a newer Fast DDS than the patched package (the ROS repository synced a new version), apt refuses the downgrade and the build fails: build a new release for that version rather than ship the stock library. The hold keeps a later `apt-get upgrade` from replacing it.

For a private repository, the download needs a read token passed as a BuildKit secret (`RUN --mount=type=secret,id=gh_token`) and the API's asset URL with `Accept: application/octet-stream`; with a public repository the plain URL above works from any build host.

## Building and testing locally

```bash
./build.sh jazzy                                   # out/jazzy/ros-jazzy-fastrtps_<ver>+husarion4_<arch>.deb
repro/closed_world.sh jazzy                        # stock: the world must close (exit 0)
repro/closed_world.sh jazzy out/jazzy/*.deb        # patched: it must stay open (exit 0)
repro/split_world.sh jazzy                         # stock: the world must split within 48 rounds (exit 0)
repro/split_world.sh jazzy out/jazzy/*.deb         # patched: it must stay whole for 8 (exit 0)
repro/blocked_sender.sh jazzy                      # stock: a sender must wedge (exit 0)
repro/blocked_sender.sh jazzy out/jazzy/*.deb      # patched: it must recover (exit 0)
```

Each reproducer reads the expected verdict from the package changelog (the subject of every patch is written into it), so a package with an older patch set is expected to fail the newer reproducers. `blocked_sender.sh` installs gdb in its container (network for apt) and needs `SYS_PTRACE`; `locks.py`, `plant.py` and `wedge.py` read and plant the port state the reproducers use and work on a live host too.

`distros.env` pins each release's base image by digest, which fixes the package version that gets patched. When the ROS repository syncs a new Fast DDS version, bump the digest and add `patches/<new version>/` in the same commit. The release workflow builds on amd64 and arm64 and publishes only when, on both, the stock package fails all three reproducers and the patched one passes them.

## When to drop this

When upstream ships the fix and it reaches the ROS apt repository for a distro, stop installing the package for that distro and remove its row from `distros.env`. The upstream reports: not filed yet (drafts for fix 1 in `upstream/`).
