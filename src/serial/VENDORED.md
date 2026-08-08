# `serial` — vendored, not a submodule

    upstream   https://github.com/tylerjw/serial.git
    branch     ros2
    commit     d8d160678aa0b31cdf467c052b954fa287cc6cdf
    subject    "position independent code" (Tyler Weaver, 2022-02-13)
    vendored   2026-08-08, clean checkout, no local modifications

## Why plain files rather than a git submodule

* **It is 612 KB.** The size argument that justifies gitignoring
  `ros2_kortex` (30 MB) does not apply here.
* **A submodule fails silently.** Anyone cloning without `--recursive` gets
  an empty directory and a CMake error that names `serial` without saying it
  was never fetched. For a build dependency this obscure that is a bad
  trade.
* **It pins a third-party fork.** `tylerjw/serial` is a personal fork kept
  alive for ROS 2; if the `ros2` branch moves or the repo disappears, a
  submodule breaks the build retroactively. Vendoring freezes the exact tree
  that is known to work.
* **The `.git` directory was removed** so this cannot become a broken nested
  repository that `git status` reports as untracked-but-unenterable.

## What needs it

`robotiq_driver` links against it. That package is currently `COLCON_IGNORE`d
(see `src/ros2_robotiq_gripper/README_BUILD.md`), so nothing in the default
build depends on this today — it is here so the gripper driver can be built
without hunting for the dependency.

## Updating it

    rm -rf src/serial
    git clone -b ros2 https://github.com/tylerjw/serial.git src/serial
    rm -rf src/serial/.git
    # then update the commit hash above
