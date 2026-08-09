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
