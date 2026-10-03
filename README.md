# fastdds-patched

Fast DDS as the ROS 2 apt repository ships it, rebuilt from the same source package with one fix that upstream does not have yet. Each package is a drop-in replacement for `ros-<distro>-fastrtps` (Humble, Jazzy) or `ros-<distro>-fastdds` (Lyrical) at the exact version our images install, with `+husarion1` appended to the version.

## The bug

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

The change is inside a private header, so the library's ABI does not change.

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

## Every process needs it

A single unpatched process in the shared-memory world keeps writing into its stale slot, so the world stays exposed. Install the package in every image whose processes share the robot's `/dev/shm` world: the driver, rosbridge, cameras, the airlock halves and anything else, including the `ros2` CLI that runs inside those containers.

## Using the packages

Each GitHub release carries the `.deb` files for every distro and architecture plus `SHA256SUMS`. Releases are private, so a build downloads with a read token passed as a BuildKit secret:

```dockerfile
ARG FASTDDS_PATCHED=v1
ARG FASTDDS_DEB=ros-jazzy-fastrtps_2.14.6-1noble.20260901.222814+husarion1_amd64.deb
ARG FASTDDS_SHA256=<from SHA256SUMS>
RUN --mount=type=secret,id=gh_token \
    curl -fsSL -H "Authorization: Bearer $(cat /run/secrets/gh_token)" -H "Accept: application/octet-stream" \
      -o /tmp/fastdds.deb "$(curl -fsSL -H "Authorization: Bearer $(cat /run/secrets/gh_token)" \
        https://api.github.com/repos/husarion/fastdds-patched/releases/tags/${FASTDDS_PATCHED} \
        | python3 -c "import json,sys; print([a['url'] for a in json.load(sys.stdin)['assets'] if a['name']=='${FASTDDS_DEB}'][0])")" \
 && echo "${FASTDDS_SHA256}  /tmp/fastdds.deb" | sha256sum -c - \
 && apt-get install -y /tmp/fastdds.deb && apt-mark hold "$(dpkg-deb -f /tmp/fastdds.deb Package)" \
 && rm /tmp/fastdds.deb
```

If the image already has a newer Fast DDS than the package (the ROS repository synced a new version), apt refuses the downgrade and the build fails: build a new release for that version rather than shipping the stock library. The hold keeps a later `apt-get upgrade` from replacing the package.

## Building and testing locally

```bash
./build.sh jazzy                                   # out/jazzy/ros-jazzy-fastrtps_<ver>+husarion1_<arch>.deb
repro/closed_world.sh jazzy                        # stock: the world must close (exit 0)
repro/closed_world.sh jazzy out/jazzy/*.deb        # patched: it must stay open (exit 0)
```

`distros.env` pins each release's base image by digest, which fixes the package version that gets patched. When the ROS repository syncs a new Fast DDS version, bump the digest and add `patches/<new version>/` in the same commit. The release workflow builds on amd64 and arm64 and publishes only when the stock package closes the world and the patched one keeps it open on both.

## When to drop this

When upstream ships the fix and it reaches the ROS apt repository for a distro, stop installing the package for that distro and remove its row from `distros.env`. The upstream report: not filed yet.
