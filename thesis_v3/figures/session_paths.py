"""Find a recording folder from its timestamp key.

The derived data files name a session only by its YYYYMMDD_HHMMSS key; the
folder on disk carries more than that. One glob, used by every figure
script, so the folder is found the same way everywhere.
"""

import glob
import os

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
SESS = os.path.join(ROOT, "recordings", "sessions")


def session_dir(key):
    hits = sorted(glob.glob(os.path.join(SESS, "*%s*" % key)))
    hits = [h for h in hits if os.path.isdir(h)]
    if len(hits) != 1:
        raise FileNotFoundError("session key %r matches %d folders under %s" % (key, len(hits), SESS))
    return hits[0]
