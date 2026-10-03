# Keep the new listener index when a SHM listener regenerates its port

`Listener::regenerate_port()` move-assigns a freshly constructed `Listener` into `*this`; the move assignment did not copy `listener_index_`, so the regenerated listener kept writing its status into its old slot index on the new port. A participant that later took that slot could die unnoticed by the port watchdog, its registration never popped any cell, and the port's ring filled until no new participant could be discovered on the host.

Fixes #<issue>.

- Copy `listener_index_` in `Listener::operator=(Listener&&)`.
- Test: <a unit test in test/unittest/transport/SharedMemTests.cpp that regenerates a port with two listeners, kills the second's process image (or simulates a frozen status on its slot) and asserts the watchdog marks the port not ok>.

Backport to 2.6.x, 2.14.x and 3.x requested: the patch applies unchanged to all three.
