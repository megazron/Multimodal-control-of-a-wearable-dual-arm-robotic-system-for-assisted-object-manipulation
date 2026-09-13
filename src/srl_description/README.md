# srl_description — the robot

`srl_dual.urdf.xacro` — two Kinova Gen3 7-DOF arms on a wearable backpack,
plus the wearer, who **is in the collision model**.

## Mount orientation is parameterised by BRACKET ANGLES, not a raw rpy triple

    mount_tilt_deg  =  60     forward tilt of the plate NORMAL from vertical
    mount_clock_deg = -150    clocking of the LEFT arm on its bolt circle

The right mount is **derived** as the exact sagittal mirror, so an asymmetric
pair cannot be written. In this file's frame `x` is lateral and `y` is
fore/aft, so a rotation about `y` is a lateral splay and **not** a forward
pitch — the previous value, `rpy="0 0.6 0"`, made exactly that mistake and
pointed both arms up and back over the shoulders. See docs/ENGINEERING_LOG.md.

    xacro src/srl_description/urdf/srl_dual.urdf.xacro mount_tilt_deg:=65

## Depends on

`kortex_description`, `robotiq_description`. Installed with `--symlink-install`,
so URDF edits take effect on the next `move_group` start — no rebuild.
