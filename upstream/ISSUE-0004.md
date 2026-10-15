# SHM transport: remove_statistics_submessage() cuts 60 payload bytes off a message of 65511 bytes or more whose byte at size - 60 is 0x80

## Summary

Since the sender-side size guard (#5742, backported to 2.14.x), `RTPSMessageGroup::send()` appends the statistics submessage only when the message is shorter than `65535 - RTPSMESSAGE_DATA_MIN_LENGTH` (65511) bytes, so that an unfragmented DATA above 64 KiB stays the last submessage. `SharedMemTransport::send()` still calls `remove_statistics_submessage()` on every message, and `get_statistics_message_pos()` decides that the message carries the submessage from a single byte, `send_buffer[size - 60] == FASTDDS_STATISTICS_NETWORK_SUBMESSAGE` (0x80). A message of 65511 bytes or more never carries the submessage, so that byte is payload, and when it happens to be 0x80 the transport sends the message 60 bytes short.

Such messages exist only on the SHM transport with a `maxMessageSize` above 65535 (a common setting for large samples over shared memory; ours is 524288). Consequences:

- An unfragmented sample between about 64 KiB and `maxMessageSize` reaches the reader 60 bytes short whenever its byte at that position is 0x80: about one random sample in 256, every sample of a constant 0x80 fill. The DATA submessage carries length 0 (it extends to the end of the message), so the reader accepts the short payload. Reliable QoS does not heal it.
- A fragmented sample exposes every DATA_FRAG message. With best effort, about 6 % of 1 MiB random samples are lost (one of their ~17 fragments is cut). Reliable heals random data by retransmission, but a payload whose byte stays 0x80 (a constant fill) never arrives.

`set_statistics_submessage_from_transport()` (UDP and TCP, through `OutputTrafficManager`) uses the same test and writes a locator, a timestamp and counters at that position. UDP datagrams stay below the threshold, but a TCP transport with a `maxMessageSize` above 65535 would overwrite 56 payload bytes in place. We found this by reading the code and did not reproduce it.

## Versions

2.14.6, 2.14.7 and the 2.14.x branch as of 2026-10-15. 2.6.12 is not affected (it has no guard and always appends the submessage). 3.6.2 is not affected in our tests: it removes the submessage by buffer identity (`remove_statistics_buffer()` on the last `NetworkBuffer`).

## Reproduction

Two participants on one host, SHM transport only, `maxMessageSize` 524288 (any `segment_size` that fits the sample), statistics enabled (the default build). Publish a `sensor_msgs/msg/Image` (any type whose payload ends in a byte sequence works) and take the serialized samples raw:

| Data | Received serialized length (expected 52 + N) |
|---|---|
| 70 000 B of 0x00, 0x80 at `data[N - 60]` | 60 short, every sample, reliable and best effort |
| 70 000 B of 0x00, 0x80 at `data[N - 61]` or `data[N - 59]` | whole |
| 70 000 B of 0x80 | 60 short, every sample |
| 1 MiB of 0x80 (fragmented) | nothing arrives, reliable or best effort |
| 100 000 B of random data | about 1 in 256 samples 60 short |

`data[N - 60]` is the message's byte at `size - 60` because the data is the last field of the sample and the DATA the last submessage. The full script is `repro/large_message.sh` in husarion/fastdds-patched (Docker, ROS 2 Jazzy base image, rclpy): stock 2.14.7 reads TRUNCATED, with the fix below WHOLE, and stock 2.6.12 and 3.6.2 read WHOLE.

## Fix

Decide by the submessage chain, not by one byte. `get_statistics_message_pos()` walks the submessages from the RTPS header the way `MessageReceiver` reads them, and only returns the position when a submessage starts exactly at `size - 60` with the statistics id and length. The one-byte test stays as the fast path, so most messages are never walked:

```diff
--- a/src/cpp/statistics/rtps/messages/RTPSStatisticsMessages.hpp
+++ b/src/cpp/statistics/rtps/messages/RTPSStatisticsMessages.hpp
@@ inline uint32_t get_statistics_message_pos(
         uint32_t pos = send_buffer_size - statistics_submessage_length;
         if (FASTDDS_STATISTICS_NETWORK_SUBMESSAGE == send_buffer[pos])
         {
-            // only succeed if the message is properly formatted
-            statistics_pos = pos;
+            using namespace eprosima::fastrtps::rtps;
+            uint32_t walk = RTPSMESSAGE_HEADER_SIZE;
+            while (walk < pos)
+            {
+                const octet id = send_buffer[walk];
+                const bool little_endian = (send_buffer[walk + 1] & 0x01) != 0;
+                const uint32_t length = little_endian ?
+                        (send_buffer[walk + 2] | (send_buffer[walk + 3] << 8)) :
+                        ((send_buffer[walk + 2] << 8) | send_buffer[walk + 3]);
+                if ((0 == length) && (INFO_TS != id) && (PAD != id))
+                {
+                    // This submessage extends to the end of the message: nothing follows it
+                    break;
+                }
+                walk += RTPSMESSAGE_SUBMESSAGEHEADER_SIZE + ((length + 3u) & ~3u);
+            }
+            if (walk == pos)
+            {
+                const bool little_endian = (send_buffer[pos + 1] & 0x01) != 0;
+                const uint32_t length = little_endian ?
+                        (send_buffer[pos + 2] | (send_buffer[pos + 3] << 8)) :
+                        ((send_buffer[pos + 2] << 8) | send_buffer[pos + 3]);
+                if (statistics_submessage_data_length == length)
+                {
+                    statistics_pos = pos;
+                }
+            }
         }
```

(plus `#include <fastdds/rtps/messages/RTPS_messages.h>` for `INFO_TS` and `PAD`). Why not something smaller:

- A length test on the transport side cannot decide it: with the submessage a message is 80 to 65570 bytes long, without it 65511 or more, so 65511 to 65570 is ambiguous both ways.
- Passing a flag from `RTPSMessageGroup` to the transport changes `TransportInterface::send()`, a public interface.
- The 3.x approach (the submessage as its own buffer) needs the gather-send transport API that 2.14 does not have.

Because the fix is in `get_statistics_message_pos()`, it also covers the UDP/TCP write path.

## Pull request text

**Statistics: identify the statistics submessage by the submessage chain before stripping or stamping it (2.14.x)**

`get_statistics_message_pos()` took a 0x80 byte at `size - 60` as proof that the message ends with the statistics submessage. Since the sender-side guard of #5742, messages of 65511 bytes or more never carry one, so on the SHM transport with a large `maxMessageSize` a payload byte of 0x80 at that position made `SharedMemTransport::send()` drop the last 60 bytes of real data (and the UDP/TCP path would stamp statistics over them). The function now walks the submessage chain and accepts only a submessage that starts at `size - 60` with the statistics id and the 56-byte length.

Fixes #<issue>.

- Test: <a unit test in test/unittest/statistics/rtps that builds a message from RTPSMessageGroup with a DATA above 65511 bytes whose byte at size - 60 is 0x80 and checks that `remove_statistics_submessage()` leaves its size alone, and that a message with the submessage is still stripped>.

Not posted: Dominik decides when.
