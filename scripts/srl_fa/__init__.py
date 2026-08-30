"""srl_fa -- the FULL AUTONOMY stack: scan the table, name what is on it,
pick it up, put it down.

SELF-CONTAINED ON PURPOSE. Nothing in this package is imported by anything
outside it, and it modifies nothing outside it. It READS the proven modules in
scripts/ (srl_fk, srl_scene, execute_pick_left, servo_pick_left) and adds what
full autonomy needs on top of them:

    fa_perception  RGB-D -> table plane, objects with colour and size, and the
                   two extrinsic-free guards that stop the hand hitting things
    fa_kin         IK on ANY link (the camera, not just the pads), and a
                   transit check that walks the whole hand over the measured
                   table
    fa_arm         one arm: joint states, motion, gripper, camera frames
    fa_scan        get the camera to a good VIEWING GEOMETRY, then scan
    fa_manip       reach, guarded descent, grasp, lift, place
    fa_prompt      a typed sentence -> an action
    fa_session     the state machine the GUI drives

The window is scripts/fa_gui.py. The headless equivalent, for testing on
hardware without a display, is scripts/fa_cli.py.
"""
__all__ = ["fa_perception", "fa_kin", "fa_arm", "fa_scan", "fa_manip",
           "fa_prompt", "fa_session"]
