#!/usr/bin/env python3
"""Subscribes to /b1 ... /b<N> and rewrites <out> every 0.5 s with one count per topic.

Usage: counter.py <out> <N>
"""
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

out, n = sys.argv[1], int(sys.argv[2])
rclpy.init()
node = Node("counter")
counts = [0] * n


def make_cb(i):
    def cb(_msg):
        counts[i] += 1
    return cb


subs = [node.create_subscription(String, f"/b{i + 1}", make_cb(i), 10) for i in range(n)]


def dump():
    with open(out + ".tmp", "w") as f:
        f.write(" ".join(map(str, counts)) + "\n")
    import os
    os.replace(out + ".tmp", out)


node.create_timer(0.5, dump)
rclpy.spin(node)
