# SHM transport: recover_blocked_processing() locks empty_cv_mutex twice and a sender loops forever

## Summary

`SharedMemGlobal::Port::get_and_remove_blocked_processing()` holds the port's `empty_cv_mutex` and calls `listener_processing_stop(i)`, which locks the same non-recursive `interprocess_mutex` again. With `BOOST_INTERPROCESS_ENABLE_TIMEOUT_WHEN_LOCKING` the inner lock times out after 1 s and its exception is caught inside `listener_processing_stop`. The slot stays marked `is_processing`, the function returns `true`, and `SharedMemManager::Port::recover_blocked_processing()` calls it again in `while (get_and_remove_blocked_processing(...))`, forever.

`recover_blocked_processing()` runs from `Port::try_push()` → `regenerate_port()` when a push throws on a port marked not ok and the port is a zombie. A participant SIGKILLed between taking a message and finishing it leaves such a slot. The next sender whose push to that port sees it marked not ok (the flag flips after `SharedMemTransport::send()`'s `cleanup_output_ports()`) loops on its sending thread while holding `RTPSParticipantImpl::m_send_resources_mutex_`. From then on nothing it sends goes out. Its discovery listener blocks in `createSenderResources()` at the next new participant, so it no longer follows the discovery port when the others regenerate it. The process stays alive and never recovers.

## Versions

2.6.12, 2.14.6, 3.6.2 and master (2026-10-04): the same two lines in `SharedMemGlobal.hpp` and `SharedMemManager.hpp`.

## Reproduction

Three publishers and a subscriber with only the SHM transport. Stop the subscriber, set `is_processing` on its own ports' listener slots (what a death mid-message leaves), then SIGKILL it. Mark its ports not ok while a publisher is inside `SharedMemTransport::push_discard()` for one of them. That publisher's thread then waits on the dead port's `empty_cv_mutex` with itself as the owner, and a restarted subscriber never hears it. The full script is `repro/blocked_sender.sh` in husarion/fastdds-patched, which uses gdb for the flip timing: WEDGED 6 of 6 on stock, 0 of 6 with the fix, on all three releases, amd64 and arm64. On a robot (17 processes, Jazzy, arm64) the same death stranded a process in 4 of 11 rounds without the fix and 0 of 23 with it.

## Fix

```diff
                         buffer_descriptor = node_->listeners_status[i].descriptor;
-                        listener_processing_stop(i);
+                        node_->listeners_status[i].is_processing = false;
                         return true;
```

and a bound of `LISTENERS_STATUS_SIZE` iterations on the loop in `recover_blocked_processing()`. Not posted: Dominik decides when.
