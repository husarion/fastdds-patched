# fastdds-patched

Fast DDS as the ROS 2 apt repository ships it, rebuilt from the same source package with three shared-memory transport fixes that upstream does not have yet. Each package is a drop-in replacement for `ros-<distro>-fastrtps` (Humble, Jazzy) or `ros-<distro>-fastdds` (Lyrical) at the exact version our images install, with `+husarion<N>` appended to the version: `+husarion1` in v1 (fix 1), `+husarion4` in v2 (fixes 1 to 3; `+husarion2` and `+husarion3` were unreleased test builds), `+husarion5` in v3 (fixes 1 to 3, with Jazzy moved to Fast DDS 2.14.7, the version the ROS repository ships since its sync of 2026-09-11).

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

Present in 2.6.12, 2.14.6, 2.14.7, 3.6.2 and master as of 2026-10-04.

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

The UGV OS release candidate of 2026-10-04 (every robot-world process on v1) split its world once more, after a graceful restart of the airlock's robot half: a second participant exec'd into the half's container died by SIGKILL with it, and two driver processes were left on unlinked segments of the discovery port with their listeners stuck in processing, their own ports removed by their peers. `repro/restart_world.sh` models that sequence in one container: root participants, a uid-10001 "half" restarted with SIGTERM, a side participant SIGKILLed 0.4 s later, and short-lived `ros2 topic echo` participants ended by SIGTERM or SIGKILL. With `LEVER=forced` (the side participant dies with its listeners marked processing, and the in-send flip of `blocked_sender.sh`), Jazzy amd64:

| Package | `restart_world.sh`, `LEVER=forced` |
|---|---|
| stock 2.14.6 | STRANDED 8 of 8: the publisher whose send was caught strands on the old discovery segments, keeps its own ports' locks after its peers removed the ports, and holds the dead port's mutex with its own thread |
| `+husarion1` (v1) | STRANDED 8 of 8, the same three marks |
| `+husarion4` (v2) | WHOLE 0 of 8 |

Without the flip, the world's own timing in the container strands no round on any package (stock 0 of 16, v1 0 of 16, `+husarion4` 0 of 158, 64 of them with `docker update --cpuset-cpus` bursts during the restart; with the processing mark alone 0 of 12 each); the lever stands in for the robot's timing, which caught the same path in 4 of 11 rounds without 0003. An earlier version of the model let the `ros2` daemon, which the CLI participants spawn, survive into the next round's world after its files were deleted; that version stranded idle participants on every package (4 of 100 rounds, none since in 226), so the model now kills the daemon every round and those rounds are no reading of the packages.

## v3: Jazzy on Fast DDS 2.14.7

The ROS repository synced Fast DDS 2.14.7 for Jazzy on 2026-09-11, after the newest `ros:jazzy-ros-base` image was built, so an image that runs `apt-get upgrade` installs 2.14.7 and can no longer take the v2 package (apt refuses the downgrade). v3 rebuilds the same three patches on 2.14.7, as `+husarion5`. Humble (2.6.12) and Lyrical (3.6.2) did not move in the repository and are rebuilt from the same sources and patches as v2.

What 2.14.7 changes, against the v2.14.6 tag: 80 source files, mostly security (the security manager, PKI-DH, the AES-GCM crypto plugin, permissions parsing), the TCP transport and its RTCP messages, a new `BaseReader.cpp`, participant and endpoint discovery, a length underflow guard in `MessageReceiver`, an IPv4 address length check, the data-sharing listener and reader pool, and Fast CDR 2.2.8. None of it touches the shared-memory transport: `src/cpp/rtps/transport/shared_mem/` and `src/cpp/utils/shared_memory/` are byte-identical between the two tags, and the ROS source package `ros-jazzy-fastrtps 2.14.7-1noble` carries them unchanged (no Debian patches). The only shared-memory-adjacent change is in the bundled Boost: `rbtree_best_fit::check_sanity()`, which `SharedMemGlobal` calls when it opens an existing port, now also rejects a free block of size zero and checks the free total as it goes. A healthy segment passes both versions; only an already corrupt one can read differently. So upstream fixed none of the three bugs, and `patches/2.14.7/` are the 2.14.6 patches byte for byte. They apply without fuzz.

The same identity answers whether a v3 process and a v2 one can share a robot world (some images still on 2.14.6 `+husarion4`, others built later on 2.14.7 `+husarion5`). The segment and port layout (`CURRENT_ABI_VERSION`, `PortNode`, the listener table), the segment, port and lock file names (`_el`, `_sl`, `sem.*_mutex`) and the flock holder file of fix 2 (`<domain>_port<N>_mutex_holder`, its lock order and its 2 s timeout) are all the same source in both. `repro/install.sh` can mix two builds in one world (`MIX_DEB`: the uid-10001 participants load the second package, together with the repository's rmw_fastrtps and typesupport rebuilt against it, and every round prints which library each participant mapped).

| Architecture | Package | closed_world.sh | split_world.sh | blocked_sender.sh |
|---|---|---|---|---|
| amd64 | stock 2.14.7 | CLOSED 134 s after the load | SPLIT at round 1 | WEDGED 6 of 6 |
| amd64 | 2.14.7 `+husarion5` | OPEN after 600 s | WHOLE 0 of 8 | RECOVERED 0 of 6 |
| arm64 | stock 2.14.7 | CLOSED 134 s after the load (2 of 3 runs, see below) | SPLIT at round 1 | WEDGED 6 of 6 |
| arm64 | 2.14.7 `+husarion5` | OPEN after 600 s | WHOLE 0 of 8 | RECOVERED 0 of 6 |

Mixed world, Jazzy amd64, 2026-10-06, root participants on 2.14.6 `+husarion4` and the uid-10001 participants (the airlock halves' role) on 2.14.7 `+husarion5`: `split_world.sh` WHOLE 0 of 24, with 16 participants on the v2 library and 2 on the v3 one in every round, and `restart_world.sh` with `LEVER=forced` WHOLE 0 of 8, with the restarted half and its side participant on v3. In the same sitting, v2 alone read WHOLE 0 of 24 and v3 alone 1 BROKEN of 24: every participant alive, one port file, no split, and the single uid-10001 newcomer saw 8 of 10 publishers. A root newcomer read as low as 6 of 10 in rounds of both controls that stayed whole, because a round turns BROKEN only when both root reads are short but on a single short uid-10001 read. This is the newcomer's 30 s listing missing publishers, the same as v2's single 9 of 10 read above, and it says nothing about the package. A first mixed model that loaded the 2.14.7 library under the base image's rmw_fastrtps (built against 2.14.6, a pairing no image has) read BROKEN 2 of 40. One of those rounds had all three newcomers at 0 of 10 and ran before the round recorded which processes were alive. It did not recur in the faithful model, so it is no reading of the packages, but it remains unexplained.

Verdict: a robot world whose processes run v2 (2.14.6 `+husarion4`) and v3 (2.14.7 `+husarion5`) side by side carries all three fixes in every process. The two share the same shared-memory code and lock protocol, and the mixed runs read as whole as either package alone.

On arm64 (the 10-core VM), the first stock run of `closed_world.sh` read OPEN: the second death went unnoticed as it should, but the discovery ring did not fill within 600 s. Two runs after it closed at 134 s (2.14.7, and 2.14.6 as a control). The release workflow's arm64 job can therefore turn red on that step now and then without saying anything about the fix. Humble and Lyrical built on arm64 with this tooling (`+husarion5`): Humble split stock at round 1 and stayed whole patched 0 of 8, and Lyrical's stock world closed at 135 s.

## Every process needs it

A single unpatched process in the shared-memory world keeps writing into its stale slot (fix 1), resets a dead owner's port mutex without the file lock (fix 2) or can spin in the recovery loop (fix 3), so the world stays exposed. Install the package in every image whose processes share the robot's `/dev/shm` world: the driver, rosbridge, cameras, the airlock halves and anything else, including the `ros2` CLI that runs inside those containers.

## Using the packages

Each release `v<N>` carries one package per distro and architecture under a stable name, `<package>-<distro>-<arch>.deb` (`fastrtps-jazzy-arm64.deb`, `fastrtps-humble-amd64.deb`, `fastdds-lyrical-amd64.deb`), plus `SHA256SUMS` and `VERSIONS` (the full package version of each file). An image pins the release and the checksum, so a rebuilt release can never slip in unnoticed:

```dockerfile
# The Fast DDS shared-memory fixes (husarion/fastdds-patched). Every image whose
# processes share the robot's /dev/shm ROS world needs it. The last three
# checks prove it at build time: the patched package is installed, its files
# are intact, and no other copy of the library exists for a process to load.
ARG FASTDDS_PATCHED=v3
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
./build.sh jazzy                                   # out/jazzy/ros-jazzy-fastrtps_<ver>+husarion5_<arch>.deb
repro/closed_world.sh jazzy                        # stock: the world must close (exit 0)
repro/closed_world.sh jazzy out/jazzy/*.deb        # patched: it must stay open (exit 0)
repro/split_world.sh jazzy                         # stock: the world must split within 48 rounds (exit 0)
repro/split_world.sh jazzy out/jazzy/*.deb         # patched: it must stay whole for 8 (exit 0)
repro/blocked_sender.sh jazzy                      # stock: a sender must wedge (exit 0)
repro/blocked_sender.sh jazzy out/jazzy/*.deb      # patched: it must recover (exit 0)
LEVER=forced CASES=4 repro/restart_world.sh jazzy                  # stock: the restart sequence strands a sender (exit 0)
LEVER=forced CASES=4 repro/restart_world.sh jazzy out/jazzy/*.deb  # patched: the world stays whole (exit 0)
```

Each reproducer reads the expected verdict from the package changelog (the subject of every patch is written into it), so a package with an older patch set is expected to fail the newer reproducers. `blocked_sender.sh` installs gdb in its container (network for apt) and needs `SYS_PTRACE`; `locks.py`, `plant.py` and `wedge.py` read and plant the port state the reproducers use and work on a live host too.

`distros.env` pins each release's base image by digest and, optionally, the stock source package version to patch (such as `2.14.7-1noble`; the binary adds a build stamp that differs per architecture). Without a version, the build patches the version the base image carries. The ROS repository can ship a newer Fast DDS before any base image carries it (Jazzy's 2.14.7 arrived after the newest `ros:jazzy-ros-base` was built), so a version field makes the build and the reproducers install that version from the repository first. The repository keeps only its newest version, so a pinned version stops building once it moves on. When the ROS repository syncs a new Fast DDS version, set the version field (or bump the digest to a base image that carries it) and add `patches/<new version>/` in the same commit. The release workflow builds on amd64 and arm64 and publishes only when, on both, the stock package fails all three reproducers and the patched one passes them.

## When to drop this

When upstream ships the fix and it reaches the ROS apt repository for a distro, stop installing the package for that distro and remove its row from `distros.env`. The upstream reports: not filed yet (drafts in `upstream/`). Fast DDS 2.14.7 does not fix any of the three.
