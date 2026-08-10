# Archive manifest

| moved | from | replaced by | commit |
| --- | --- | --- | --- |
| archive/scripts/srl_hud_20260810_pre_holographic.py | scripts/srl_hud.py | superseded by the holographic MasterArmSchematic / RobotSchematic rewrite (Part 4). The predecessor is kept because its Strip, Card and ReadyPanel classes are unchanged and its schematic is the version every clip before 2026-08-10 was captured against. | (this commit) |
| archive/recordings/verification_20260810/ | recordings/verification/ | superseded by the 2026-08-10 re-record against the verified geometry (bench in the planning scene, objects resting at the near edge, wrist derived from the object rather than used as the object). These were recorded against an EMPTY planning scene with objects floating 0.107-0.357 m above a decorative bench. Kept because several findings cite them -- the mode axis, the VR half-scale result, the isolation rule and the verifier's calibration history are all still sound. See GEOMETRY_NOTE.md in that directory. | (this commit) |
