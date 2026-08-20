# VR SESSION CHECKLIST

**One action per line. Tick as you go. You do not need to have read anything
else.**

If you only remember one thing: **the button is `START VR TELEOP`, on the RUN
tab.** Everything below is what to do when it stops.

---

## BEFORE THE HEADSET GOES ANYWHERE

```
[ ]  1.  Open a terminal.
[ ]  2.  Type:   bash ~/kortex_ws/scripts/start_gui.sh
[ ]  3.  A dark window opens.
         PASS: it opens. FAIL: see F1.
[ ]  4.  Click the RUN tab on the left.
[ ]  5.  Put the headset on its shelf, lenses pointing at where your hands
         will be, about 1.2 m away, chest height, on something nobody leans
         on.
[ ]  6.  Tape a folded scrap of paper over the proximity sensor (the small
         hole between the lenses). This keeps it awake off-head. It comes
         off in one second and changes no settings.
```

## THE BUTTON

```
[ ]  7.  Press  START VR TELEOP.
[ ]  8.  Watch the twelve lines under it fill in, top to bottom.
         PASS: each turns to "OK". Takes up to two minutes the first time,
               because it starts the simulation.
         FAIL: the first line that says "FIX" has a button under it in plain
               words. Press that button. It runs the repair and starts again
               from the top. See F2 if you want to know what it did.
[ ]  9.  Stop when the headline says:
             everything on this machine is ready -- connect the headset
         That is the correct end with the headset not yet connected.
```

## THE HEADSET

```
[ ] 10.  Press  open the headset page.  Note the address it prints.
[ ] 11.  Put the headset on, open its own browser, type that address.
[ ] 12.  Accept the one security warning. It is this machine's own
         certificate.
         PASS: the page loads and shows "immersive-vr supported".
         FAIL: see F3.
[ ] 13.  Press ENTER VR on the page.
         (Or ENTER WITH PASSTHROUGH if you want to see the room. It changes
         nothing about the robot.)
[ ] 14.  Take the headset off and put it back on the shelf. Nobody wears it.
[ ] 15.  Pick up both controllers.
[ ] 16.  Press  START VR TELEOP  once more.
         PASS: the headline says  READY TO OPERATE.
         FAIL: see F4.
```

## BEFORE YOU DRIVE ANYTHING

```
[ ] 17.  In a terminal:   bash scripts/vr_measure_session.sh yaw
[ ] 18.  Do what it asks: move your hand ~40 cm straight TOWARDS the wearer,
         then ~40 cm to your OWN RIGHT.
         PASS: it prints an angle and writes it. Near 180 is expected --
               you are facing the wearer.
         FAIL: it refuses. See F5. Do not continue without this.
[ ] 19.  Squeeze and hold the GRIP (middle finger). Move your hand 5 cm.
         PASS: the SIMULATED arm moves the same way you did.
         FAIL: release the grip. See F6.
```

**STOP HERE unless the arms are being powered today.** Everything above is
simulation. The real arms are a separate procedure:
`docs/system/VR_REAL_ARM_RUN.md`, and it starts with an observer and an
e-stop test.

---

# WHEN IT FAILS

## F1 — the window does not open

```
[ ]  Read what the terminal printed.
[ ]  "already open"     -> use the window you have, or add:  --anyway
[ ]  anything else      -> type:  bash scripts/diagnostics.sh
```

## F2 — a line says FIX

Press the button underneath it. That is the whole answer. The nine repairs
and what each does:

| the line says | the button does |
| --- | --- |
| this window is not set up | reopens the window with the right settings |
| a previous run left blocks of memory | clears them, waits five seconds, restarts the helper that lists what is running |
| asking what is running does not come back | restarts that helper |
| something is running twice | stops the newest copy, keeps the first |
| something left over is claiming to be the robot | stops the leftover only |
| the simulation did not finish starting | shows you what it printed |
| the simulation IS running and cannot be seen | restarts that helper |
| there is no security certificate | makes one |
| the certificate is for a different address | makes a new one |
| the connection point is already held | stops what is holding it |
| the part that talks to the headset stopped | shows you what it printed |
| the mapping is holding settings from an earlier run | clears them |

**After any repair, the sequence restarts from the top by itself.** You do not
press anything.

**If the same line fails twice**, the repair did not take. Read the small grey
text under the line -- it says what was actually found -- and stop. Do not
press it a third time.

## F3 — the page will not load in the headset

```
[ ]  Is the headset on the same wifi as this machine?  Fix that first.
[ ]  Does the address start with  https://  ?  It must.
[ ]  Press  START VR TELEOP  again and check the certificate line is OK.
[ ]  Still nothing: the Windows firewall. On Windows, in an ADMIN
     PowerShell, run BOTH lines that  bash scripts/start_vr_wifi.sh --check
     prints. There are two firewalls in mirrored mode and the second one is
     the one people miss.
```

## F4 — it still says "connect the headset" after ENTER VR

```
[ ]  Is the page still open in the headset? Poses only flow inside a
     session. Backgrounding the browser stops them.
[ ]  Did the headset go to sleep? Check the tape over the proximity sensor.
[ ]  Are you holding the controllers, awake, in view of the headset?
[ ]  Press  START VR TELEOP  again.
```

## F5 — the yaw calibration refuses

It refuses rather than writing a confident wrong number. Two reasons:

```
[ ]  The motion was too small (under 10 cm) or too far off horizontal
     (more than 35 degrees). Do it again, bigger and flatter.
[ ]  The shelf is tilted. Level it and repeat.
```

**Do not skip this and drive anyway.** You are facing the wearer, so your
"away from me" is the robot's "towards its own front". Without the angle the
arm moves in a direction nobody chose.

## F6 — the arm moves the wrong way

```
[ ]  Release the grip immediately.
[ ]  Do NOT adjust anything by hand.
[ ]  Re-run:  bash scripts/vr_measure_session.sh yaw
[ ]  If it moves the wrong way again, stop and write down which physical
     arm moved when you drove which controller. There is an open question
     in this project about whether the arm named "right" is on the wearer's
     right, and that is how it would show up.
```

## F7 — the arm does not move at all

In this order:

```
[ ]  Are you holding the grip? It is the middle-finger trigger.
[ ]  Is that controller visible to the headset? Covering it freezes that
     arm on purpose.
[ ]  Did the observer stop confirming? Two seconds of silence freezes
     everything. (Real-arm sessions only.)
[ ]  Was the headset knocked? A moved reference freezes and does not
     un-freeze on its own. Ask for a re-base and re-run the yaw step.
```

---

# WHEN YOU ARE DONE

```
[ ] 1.  Release the grip.
[ ] 2.  Press  stop VR.  The simulation stays up on purpose, so starting
        again is quick.
[ ] 3.  Close the GUI window.
[ ] 4.  Take the tape off the headset.
```

If you touched the real arms today, use the teardown in
`docs/system/VR_REAL_ARM_RUN.md` §19 instead -- the order matters there and
Ctrl-C once, not twice, is the difference between a clean session and one
that blocks tomorrow.
