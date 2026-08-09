# Embedding RViz in the GUI: three routes, measured

Investigated 2026-08-09 on this machine (WSLg, XWayland, Qt 5, ROS 2 Jazzy).

## Route 1 — X11 reparenting into a Qt widget. **WORKS, and is what ships.**

`QX11EmbedContainer` was removed in Qt5. The modern equivalent is
`QWindow.fromWinId()` wrapped by `QWidget.createWindowContainer()`, which
reparents a *foreign* window. `rviz2` is an X11 client under XWayland, so it
has a real window id and this is well defined.

**Verified from pixels, not from a return code.** Under Xvfb the host window's
own label renders above and the complete RViz UI renders inside the container,
reporting 31 fps. A screenshot is the evidence; `container.isVisible()`
returning true is not, because a container can be visible and empty.

| | |
| --- | --- |
| host frame time with RViz embedded | **0.45 ms, median 0.50, p95 0.76** against a 100 ms budget |
| RViz frame rate inside the container | 31 fps, its own figure |
| time for the RViz window to appear | ~1.0 s |

Costs, stated honestly:

* `rviz2` stays a **separate process**, so the host cannot style it and it
  flashes briefly as its own window between mapping and reparenting.
* That separateness is also the main benefit: **an RViz crash does not take
  the GUI down**, which matters because the GUI is the tool used to diagnose
  everything else.
* Needs an X11 or XWayland session. On a pure-Wayland host there is no window
  id to reparent, so the GUI falls back with a stated reason rather than an
  empty panel.

## Route 2 — `librviz` as a real Qt widget. Not attempted.

The headers exist (`/opt/ros/jazzy/include/rviz_common/`), so this is
possible. It is not free:

* there are **no usable Python bindings for rviz2**, so this means a new C++
  Qt package and a build step in the GUI's path;
* RViz then shares the GUI's process, so an RViz crash takes the operations
  console with it.

Route 1 already works, so this buys tighter integration for a real cost. Worth
revisiting only if the flash between mapping and reparenting becomes annoying.

## Route 3 — render to an image and display frames. Fallback only.

Proven to work in this repository already: the clip pipeline runs `rviz2` on
Xvfb and captures with ffmpeg, and that is exactly this route. It is kept as
the documented fallback for a headless host.

Costs: an extra Xvfb and an encode plus decode per frame, and **it loses
interaction entirely** — the operator cannot orbit the camera. Strictly worse
than route 1 wherever route 1 is available.

## Helvetica

Helvetica is a licensed Adobe face and is **not installed here**; the usual
free clone, URW Nimbus Sans, is not present either. Liberation Sans is
metrically compatible with Arial, which is metrically compatible with
Helvetica, so it has the same advance widths and a layout designed for one
does not reflow in the other.

`~/.config/fontconfig/fonts.conf` aliases `Helvetica` to it with a strong
binding, at user level so no root is needed. Verified:

    fc-match Helvetica       -> LiberationSans-Regular.ttf
    fc-match Helvetica:bold  -> LiberationSans-Bold.ttf

The GUI still *requests* `Helvetica` by name so the intent stays visible in
the source.

## Why Qt rather than the existing Dear PyGui console

RViz is a Qt application, so its window can become a panel rather than a
second window. Qt resolves fonts through fontconfig, so "Helvetica" is a
request the toolkit can satisfy. And Dear PyGui **segfaulted on import**
during this work, intermittently, which is not a property one wants in the
tool used to diagnose everything else.

---

# CORRECTION AND EXTENSION (2026-08-09, Part 4): EMBEDDING DOES NOT CLIP HERE

Route 1 above says reparenting "works, and is what ships". Building the
dual-view GUI on it showed that claim is **too strong for this host**, and the
correction matters more than the original result.

## What actually happens

`QWindow.fromWinId()` + `QWidget.createWindowContainer()` does create a
container and Qt *will* move the foreign window when told to. What it does
**not** do here is CLIP the window to the container. Measured, from
screenshots rather than return codes:

| | |
| --- | --- |
| Qt layout underneath | **correct** -- the GUI's own geometry dump put the panels at controls (8,49)-(478,983), commanded (487,75)-(1180,857), actual (1194,75)-(1887,857), divergence (482,868)-(1892,983) |
| where rviz2 actually painted | **(0,0) at 1595x995**, over the banner, the wrist cameras, every indicator group and the divergence readout |
| with TWO instances | they paint over each other as well as over the host |

Positioning the foreign window in top-level coordinates rather than at (0,0)
moved it to roughly the right place and raised the second panel's rendered
content from 36 to 292 distinct colours -- so the handle is real and the moves
take effect -- but **no amount of positioning produces clipping**, and without
clipping an oversized RViz still covers its neighbours.

## Why, and what would fix it

Clipping a reparented client is the window manager's job, and **no window
manager is installed on this machine** (checked: openbox, matchbox, i3,
metacity, xfwm4, fluxbox, jwm, twm, marco, mutter, icewm, blackbox -- none
present, and `sudo` needs a password so none can be installed unattended).
The earlier "31 fps, renders inside the container" result was taken with a
host whose RViz panel occupied nearly the whole window, where covering the
neighbours is invisible because there are no neighbours to cover.

So the honest statement is: **X11 reparenting embeds and positions, but a
window manager is required for it to be usable as a PANEL beside other
content.** That is a host requirement, not a code fix.

## What ships instead

RViz runs as its own top-level window by default, carrying a **ghost config**:
two `RobotModel` displays in one scene, the commanded arm solid on
`/robot_description` and the actual arm translucent (alpha 0.45) on
`/real/robot_description` with `TF Prefix: real_`.

This is the single-RViz fallback the Part 4 brief asked to be proposed if the
dual panels degraded, and it turns out to be the better answer to the question
the panels existed for:

* the operator sees the **gap** in one 3-D scene instead of comparing two
  viewports by eye;
* it costs one RViz instead of two;
* it needs no reparenting, so it works on any host.

`--embed-rviz` opts back into reparenting, and `--dual-rviz` into two panels,
for a host with a window manager. Both are documented with this caveat.

## THE REAL ARM IS NOT ON /real/tf, and nothing publishes that topic

The brief specifies the actual arms come from "/real/joint_states and
/real/tf". The first exists; **the second does not**. Checked against a
running mock stack, the only `real` topics are:

```
/real/joint_states
/real/left_arm_controller/joint_trajectory
/real/right_arm_controller/joint_trajectory
/real_status_left  /real_status_right
/realmock/robot_description
```

Both real launches (`real_arms.launch.py:83`, `mock_real.launch.py:57`) run
`robot_state_publisher` with `namespace="real"` and `frame_prefix="real_"`,
which puts the real arm's transforms in the **shared /tf** under prefixed
frame names joined to `world` by a static transform. One tree with two robots
in it, not two trees.

Subscribing to `/real/tf` would have produced a permanently empty ACTUAL panel
that looked exactly like an embedding failure -- so this is recorded here
rather than left to be rediscovered.
