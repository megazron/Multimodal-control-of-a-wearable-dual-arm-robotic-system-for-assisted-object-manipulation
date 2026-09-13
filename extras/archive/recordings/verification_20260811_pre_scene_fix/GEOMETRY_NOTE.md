# SUPERSEDED GEOMETRY -- do not cite any number from these clips

Archived 2026-08-11, before the mode-06 re-record at `825408f`.

These four clips were recorded against a scene that has since been corrected
in four ways, each of which changes what is on screen:

  * **T0 was handed the bench, the bin and a circuit box** it does not use,
    and its three targets were sampled from a bench-derived 200 x 80 mm band.
    They landed 76 mm apart and the whole task was 0.219 m of commanded
    travel. T0 now has NO furniture and three distinct directions per arm,
    0.698 m of travel.
  * **T1 had no coloured planes at all**, so there was nothing to match a cube
    to and a wrong-colour placement was not scoreable from a frame.
  * **T2's tray was drawn on ONE arm** -- a 560 mm tray centred on the left
    gripper's pads, spanning from 30 mm past the right gripper to 280 mm
    beyond the left one -- and it was RELEASED on the first tick of every clip
    because the knuckle was not yet known. Its carry was also 80 mm; the band
    has since been measured clear to 1.70 and the path runs to 1.60.
  * **T3 drew two circuit boxes** and none of its four measurement points.

Worse than any of that: the run these came from gated every capture on
TIMEOUT, so `graph.moved()` never fired and the arm did not travel. The pad
distances in their `scene_events.json` (0.4234 m, 0.6895 m) are not mis-aimed
grasps, they are an arm that never left home.

Kept for the diagnosis, not for the pictures.
