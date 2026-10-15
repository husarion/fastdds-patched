"""One case of large_message.sh: an rclpy publisher or subscriber of sensor_msgs/Image.

large.py pub|sub <topic> <data bytes> <fill hex> <override index|-> <reliable|besteffort> <count> <hz>

The publisher sends <count> Images whose data is <data bytes> of <fill>, with the byte
at <override index> set to 0x80, once the subscriber has matched (and 2 s more: SHM
discovery is asymmetric), then stays 4 s so reliable repairs can finish. The
subscriber takes the raw CDR (a sample cut short may not deserialize, and would
vanish silently) and prints one line: how many arrived, how many are whole (the
exact serialized length, and every field and data byte equal to what was sent),
how many are short, and the lengths it saw. Both sides keep <count> samples of
history, so a reliable writer never overwrites an unacknowledged sample.
"""

import sys
import time
from collections import Counter

import rclpy
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import Image

role, topic, nbytes, fill, override, rel, count, hz = sys.argv[1:9]
nbytes, count, hz = int(nbytes), int(count), float(hz)


def sample():
    m = Image()
    m.header.frame_id = 'large'
    m.height = 1
    m.width = nbytes
    m.step = nbytes
    m.encoding = 'mono8'
    d = bytearray([int(fill, 16)]) * nbytes
    if override != '-':
        d[int(override)] = 0x80
    m.data = bytes(d)
    return m


qos = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=count,
    reliability=ReliabilityPolicy.RELIABLE if rel == 'reliable' else ReliabilityPolicy.BEST_EFFORT,
)
rclpy.init()
node = rclpy.create_node('large_' + role)
msg = sample()
want = serialize_message(msg)

if role == 'pub':
    pub = node.create_publisher(Image, topic, qos)
    t0 = time.time()
    while pub.get_subscription_count() == 0 and time.time() < t0 + 30:
        time.sleep(0.05)
    if pub.get_subscription_count() == 0:
        print('pub: no subscriber matched in 30 s', flush=True)
        sys.exit(1)
    time.sleep(2)
    for _ in range(count):
        pub.publish(msg)
        time.sleep(1.0 / hz)
    time.sleep(4)
    sys.exit(0)

lens = []
whole = 0
garbled = 0


def on_raw(raw):
    global whole, garbled
    lens.append(len(raw))
    if len(raw) != len(want):
        return
    try:
        got = deserialize_message(raw, Image)
    except Exception:  # noqa: BLE001  a sample that does not deserialize is not whole
        garbled += 1
        return
    if (got.header.frame_id, got.height, got.width, got.step, got.encoding) == (
        msg.header.frame_id, msg.height, msg.width, msg.step, msg.encoding
    ) and bytes(got.data) == bytes(msg.data):
        whole += 1
    else:
        garbled += 1


sub = node.create_subscription(Image, topic, on_raw, qos, raw=True)
deadline = time.time() + float(sys.argv[9]) if len(sys.argv) > 9 else time.time() + 60
while time.time() < deadline and len(lens) < count:
    rclpy.spin_once(node, timeout_sec=0.2)
short = sum(1 for n in lens if n < len(want))
print(
    f'received {len(lens)} of {count}, whole {whole}, short {short}, garbled {garbled}, '
    f'expected length {len(want)}, lengths {sorted(Counter(lens).items())[:4]}',
    flush=True,
)
