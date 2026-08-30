#!/usr/bin/env python3
"""Re-export shim: the smoothing implementation moved to `srl_teleop.smoothing`.

    python3 -m srl_vr_teleop.vr_smoothing --self-test

MOVED 2026-08-27, NOT FORKED. The master-arm path (`master_pose_node`) needed
the same 1-Euro filter this module shipped on 2026-08-26, and the dependency
arrow only runs one way (HARD CONSTRAINT 7): `srl_teleop` imports nothing
in-repo, while this package already imports `srl_teleop`
(vr_gripper_node.py:30, vr_safety_node.py:136). So the implementation lives on
that side of the arrow and this module re-exports it, name for name --
`test_vr_smoothing_shim.py` pins the export list, so the move cannot silently
drop a name a VR consumer was using.

Everything below is the same objects, importable under the old paths:
`vr_pose_mapper`, `test_vr_smoothing.py`, `measure_vr_smoothing.py` and
`make_results.py` all keep working unchanged.
"""
from srl_teleop.smoothing import (  # noqa: F401
    TWO_PI,
    LegacyEma,
    LowPass,
    OneEuro,
    OneEuroQuat,
    _Passthrough,
    _self_test,
    alpha_for,
    make,
    q_angle,
    q_canon,
    q_norm,
    q_slerp,
)

if __name__ == "__main__":
    import sys
    if "--self-test" in sys.argv:
        sys.exit(_self_test())
    print(__doc__)
