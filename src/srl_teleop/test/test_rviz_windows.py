#!/usr/bin/env python3
"""The RViz window filter, against the tree that broke it.

The sample below is the real `xwininfo -root -tree` output measured on WSLg
on 2026-08-22, when the GUI embedded the SELECTION OWNER into the commanded
panel and every check passed against an empty picture. It is the known-answer
input for the filter, and it is the reason this lives in one module now: the
window the GUI embeds and the window the VR bring-up waits for must be the
same window.
"""
from srl_teleop import rviz_windows as R

TREE = """
  0x600106 "/home/x/srl_commanded.rviz - RViz": ("rviz2" "rviz2")  1600x1000+38+59  +135+45
  0x600004 "Qt Selection Owner for rviz2": ()  549x819+0+0  +0+0
  0x600008 "rviz2": ()  1x1+0+0  +0+0
  0xa00004 (has no name): ()  1568x914+0+0  +0+0
  0x800003 "Terminal": ("gnome-terminal-server" "Gnome-terminal")  900x600+0+0  +0+0
"""


def test_only_the_window_rviz_draws_in_is_returned():
    assert R.parse(TREE) == {0x600106}


def test_the_selection_owner_is_not_a_viewport():
    """The measured defect: `sorted(new)[0]` picked 0x600004."""
    assert 0x600004 not in R.parse(TREE)


def test_a_starting_stub_is_not_mistaken_for_the_main_window():
    """rviz2 builds transient windows while starting and its real one after,
    as a SEPARATE toplevel. Embedding a stub leaves the real view outside
    the GUI, which is what happened at a 1.5 s deadline. The main window is
    the one titled "<config> - RViz"; the stubs are not.
    """
    stub = ('  0x600200 "rviz2": ("rviz2" "rviz2")  400x287+0+0  +0+0\n'
            '  0x600201 (has no name): ("rviz2" "rviz2")  100x100+0+0  +0+0\n')
    assert R.parse(stub) == set()


def test_nothing_is_a_window_when_there_is_no_rviz():
    assert R.parse("") == set()
    assert R.parse(None) == set()


def test_no_x_server_is_an_empty_answer_and_not_a_crash():
    """`real_windows()` is called from a bring-up step that must keep going
    on a headless box."""
    assert isinstance(R.real_windows(), set)
