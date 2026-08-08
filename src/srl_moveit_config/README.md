# srl_moveit_config — MoveIt configuration

TRAC-IK kinematics, controllers, and the SRDF.

## The SRDF is load-bearing for operator safety

Two exclusion groups matter and must not be widened casually:

- 36 **mount-adjacent** exclusions: `{left,right}_{base,shoulder,half_arm_1}`
  vs torso / harness / backpack / mount_plate / pads. The arm bases are bolted
  there; without these every pose is trivially in collision.
- 8 **proximal-vs-wearer's-upper-arm** exclusions, added with the 2026-08-05
  mount fix. These cover a *static* interference set by the mount position —
  no joint moves `base_link`, so no planner can avoid it. Recorded as a
  mechanical action item in CLAUDE.md rather than hidden.

**Everything distal of `half_arm_1` stays enabled.** That is what actually
keeps the arms off the operator, and its measured home clearance is 0.224 m.

    ros2 launch srl_moveit_config demo.launch.py use_rviz:=false
