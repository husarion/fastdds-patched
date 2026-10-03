# SHM transport: a regenerated listener keeps its old slot index, so a later participant death goes unnoticed and the discovery port fills

## Summary

`SharedMemManager::Listener::regenerate_port()` creates a new `Listener` (whose constructor registers on the regenerated port and stores the new slot in `listener_index_`) and move-assigns it into `*this`. `Listener::operator=(Listener&&)` copies `global_listener_`, `global_port_`, `shared_mem_manager_` and `is_closed_`, but not `listener_index_`. The surviving listener keeps its old index and from then on writes its liveness status into a slot on the new port that is not its own.

A participant that later registers on that slot can then die without the port watchdog noticing, because the survivor keeps the slot's status moving. Its registration is never removed, every later push leaves a cell it never pops, the port's ring fills, and every further push to the port fails ("Port full", logged at Info) while `is_port_ok` stays true. Participants already matched keep communicating over their unicast ports. A participant that starts afterwards on the same host is never discovered and discovers no one.

## Versions

Present in every tag we checked: v2.0.0, v2.1.0, v2.3.0, v2.5.0, v2.6.0-v2.6.12, v2.14.0-v2.14.7, v3.0.0-v3.6.2 and master (2026-10-03).

## Reproduction

Shared-memory-only participants on one host (a profile with only an SHM transport), any distro's `ros2 topic pub` and `ros2 topic list`:

1. Start three publishers A, B, C.
2. SIGKILL A. The watchdog notices and the survivors regenerate the discovery port (its file in `/dev/shm` gets a new inode).
3. Start a publisher P, then SIGKILL it. The port is not regenerated: P's death goes unnoticed.
4. Start ten more publishers. Within 100-140 s a new `ros2 topic list --no-daemon` no longer sees B's topic; B and C keep talking to each other.

With the one-line fix below, P's death is noticed (the port regenerates again) and the world stays open for 600 s of the same load. We reproduced both outcomes on Humble (2.6.12), Jazzy (2.14.6) and Lyrical (3.6.2) packages on amd64 and arm64, and on a robot whose real driver world closed 11 times out of 11 under two SIGKILLs and a restart, and 0 of 11 with every process on the patched library.

## Fix

```diff
--- a/src/cpp/rtps/transport/shared_mem/SharedMemManager.hpp
+++ b/src/cpp/rtps/transport/shared_mem/SharedMemManager.hpp
@@ Listener& operator = (Listener&& other)
             global_port_ = other.global_port_;
             other.global_port_.reset();
+            listener_index_ = other.listener_index_;
             shared_mem_manager_ = other.shared_mem_manager_;
             is_closed_.exchange(other.is_closed_);
```

The moved-from listener's destructor unregisters only while it holds a port, and its `global_port_` is reset by the move, so the new slot is not released twice.

We would ask for backports to the 2.6.x, 2.14.x and 3.x branches, since every ROS 2 distribution in support ships an affected version.
