#!/usr/bin/env python3
"""Read the REAL gripper position out of every VR pilot bag.

The 20 Hz trail never logged a gripper command (grip_cmd_* is empty in every
session), so whether an operator actually closed the gripper on an object
can only be read from the bag: /real/gripper_<hand> (Float64MultiArray,
published by kortex_highlevel_bridge from the arm's own gripper feedback).
This script reads only those two topics from each bag and writes one JSON
line per session with the raw range, the fraction of samples closed and the
number of distinct closures, so the pick outcome can be stated from the
record rather than from memory.

    source /opt/ros/jazzy/setup.bash
    python3 thesis_v3/figures/extract_pilot_gripper.py  [session_dir ...]

Output: thesis_v3/figures/pilot/data/gripper_by_session.jsonl (appended,
one line per session; a session already present is skipped).
"""

import json
import os
import sys
import time

import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import Float64MultiArray

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
SESS = os.path.join(ROOT, "recordings", "sessions")
OUT = os.path.join(HERE, "pilot", "data", "gripper_by_session.jsonl")
TOPICS = ["/real/gripper_left", "/real/gripper_right"]
CLOSED = 0.5  # fraction of the observed range; the closure count uses this


def bag_path(d):
    for cand in ("bag/bag_0.mcap", "bag/bag_0.mcap.zstd"):
        p = os.path.join(d, cand)
        if os.path.exists(p):
            return p
    return None


def scan(d):
    p = bag_path(d)
    if p is None:
        return {"session": os.path.basename(d), "error": "no bag"}
    r = rosbag2_py.SequentialReader() if p.endswith(".mcap") else rosbag2_py.SequentialCompressionReader()
    r.open(rosbag2_py.StorageOptions(uri=p, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    r.set_filter(rosbag2_py.StorageFilter(topics=TOPICS))
    series = {t: [] for t in TOPICS}
    first = {}
    t0 = time.time()
    while r.has_next():
        topic, raw, stamp = r.read_next()
        m = deserialize_message(raw, Float64MultiArray)
        if topic not in first:
            first[topic] = list(m.data)
        series[topic].append((stamp * 1e-9, list(m.data)))
    out = {"session": os.path.basename(d), "bag": os.path.relpath(p, ROOT), "scan_s": round(time.time() - t0, 1),
           "first_message": first, "hands": {}}
    for t, s in series.items():
        hand = t.rsplit("_", 1)[1]
        if not s:
            out["hands"][hand] = {"n": 0}
            continue
        ts = [a for a, _ in s]
        rec = {"n": len(s), "t_span_s": round(ts[-1] - ts[0], 1), "len_data": len(s[0][1])}
        # layout from kortex_highlevel_bridge._publish_gripper:
        # [target, measured, error], normalised 0..1, NaN where unknown.
        for k, name in ((0, "target"), (1, "measured")):
            pos = [v[k] for _, v in s if len(v) > k and v[k] == v[k]]  # drop NaN
            if not pos:
                rec[name] = {"n": 0}
                continue
            lo, hi = min(pos), max(pos)
            rng = hi - lo
            closed = [(x - lo) > CLOSED * rng for x in pos] if rng > 1e-6 else [False] * len(pos)
            closures = sum(1 for i in range(1, len(closed)) if closed[i] and not closed[i - 1])
            rec[name] = {"n": len(pos), "min": lo, "max": hi, "range": rng,
                         "closed_frac": sum(closed) / len(closed), "closures": closures}
        out["hands"][hand] = rec
    return out


def main():
    done = set()
    if os.path.exists(OUT):
        done = {json.loads(l)["session"] for l in open(OUT) if l.strip()}
    dirs = sys.argv[1:] or [os.path.join(SESS, s["session"]) for s in
                            json.load(open(os.path.join(HERE, "pilot", "data", "vr_study_sessions.json")))]
    with open(OUT, "a") as f:
        for d in dirs:
            if os.path.basename(d) in done:
                continue
            print("scanning", os.path.basename(d), flush=True)
            try:
                rec = scan(d)
            except Exception as e:  # keep going; one bad bag must not lose the rest
                rec = {"session": os.path.basename(d), "error": repr(e)}
            f.write(json.dumps(rec) + "\n")
            f.flush()
            print("  ", {k: v for k, v in rec.items() if k in ("scan_s", "hands", "error")}, flush=True)


if __name__ == "__main__":
    main()
