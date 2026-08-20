"""Passthrough changes the session mode and NOTHING on the arm's path.

WHY THIS IS A TEST AND NOT A NOTE. The claim being made is "passthrough adds
no latency to the robot", and that claim is only true because the pose path is
literally the same code in both modes. If a future edit gave the AR path its
own frame loop, its own sampling, or its own socket, the claim would quietly
become false and nothing would fail. So the shared-path property is asserted
rather than described.

The second thing asserted is the honesty of the AR report. Requesting
`immersive-ar` does NOT guarantee passthrough: a headset can grant the session
and still composite opaque, and the operator would then be looking at a black
room while the page believed it had shown them the lab. `environmentBlendMode`
is the only thing that distinguishes those, so the client must read it and say
so.

No browser and no headset: this reads the shipped client the bridge serves.
"""
import os
import re

import pytest

WEB = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   "web", "vr_client.html")


@pytest.fixture(scope="module")
def src():
    with open(WEB) as fh:
        return fh.read()


def test_both_modes_are_offered_and_both_are_probed_first(src):
    """A button that starts a session the headset cannot grant fails INSIDE
    the headset, where there is no console to read."""
    assert "immersive-vr" in src and "immersive-ar" in src
    assert src.count("isSessionSupported('immersive-ar')") == 1
    assert "$('enter-ar').disabled = !ok" in src


def test_one_entry_function_serves_both_modes(src):
    """THE LATENCY CLAIM, made structural.

    `requestSession` must appear exactly once. Two call sites would mean two
    paths, and two paths can diverge without anything failing."""
    assert src.count("requestSession(") == 1, (
        "requestSession appears %d times -- the VR and passthrough paths have "
        "separated, and 'passthrough adds no latency' is no longer something "
        "this file can support" % src.count("requestSession("))
    assert "async function enterXR(mode)" in src
    for handler in ("$('enter').onclick", "$('enter-ar').onclick"):
        assert handler in src


def _strip_comments(js):
    """Drop // line comments and /* */ blocks.

    The tests below assert what the CODE does. Without this they assert what
    the comments SAY, and the first run duly failed on a comment explaining
    why the clear has alpha 0 -- prose about passthrough inside the frame
    loop, which is exactly the thing worth writing down and not at all the
    thing being forbidden.
    """
    out, i, n = [], 0, len(js)
    while i < n:
        if js.startswith("//", i):
            j = js.find("\n", i)
            i = n if j < 0 else j
        elif js.startswith("/*", i):
            j = js.find("*/", i + 2)
            i = n if j < 0 else j + 2
        else:
            out.append(js[i])
            i += 1
    return "".join(out)


def _function_body(src, header):
    """The text between a function's opening brace and its match.

    Sliced by BRACE MATCHING rather than by "up to the next thing I expect",
    which is how the first version of this test read the whole rest of the
    file and failed on code that was fine.
    """
    i = src.index(header)
    j = src.index("{", i)
    depth, k = 0, j
    while k < len(src):
        if src[k] == "{":
            depth += 1
        elif src[k] == "}":
            depth -= 1
            if depth == 0:
                return src[j:k + 1]
        k += 1
    raise AssertionError("unbalanced braces after %r" % header)


def test_the_frame_loop_is_not_forked_by_mode(src):
    """One rAF callback, and it must not branch on the session mode.

    A per-mode frame loop is the specific way this would go wrong: the AR
    path would gain a compositor wait, or drop a sample, and the poses would
    slow down only in the mode nobody measures.
    """
    assert src.count("function onXRFrame") == 1
    body = _strip_comments(_function_body(src, "function onXRFrame"))
    assert len(body) > 200, "sliced the wrong thing"
    for forbidden in ("immersive-ar", "immersive-vr", "environmentBlendMode"):
        assert forbidden not in body, (
            "the frame loop mentions %r -- the two modes no longer share the "
            "pose path" % forbidden)


def test_an_opaque_ar_session_is_reported_as_a_failure(src):
    """The case where everything succeeds and the operator sees black."""
    assert "environmentBlendMode" in src
    assert "opaque" in src
    m = re.search(r"mode === 'immersive-ar' && blend === 'opaque'", src)
    assert m, "nothing checks for an AR session that composites opaque"
    after = src[m.end():m.end() + 400]
    assert "NO " in after or "no passthrough" in after.lower()


def test_the_clear_stays_transparent(src):
    """clearColor with alpha 1 paints the room out, and an AR session that
    shows black is indistinguishable from passthrough not working."""
    assert "gl.clearColor(0, 0, 0, 0)" in src
    assert src.count("clearColor(") == 1


def test_nothing_about_passthrough_reaches_the_protocol(src):
    """The bridge parses a fixed 14-float protocol. If the client started
    sending a mode or a blend field, the bridge would have to learn about
    display technology -- which is not its job, and is how a transport grows
    a second reason to change.

    Checked on the frame body, which is the only thing that builds an
    outgoing message."""
    body = _strip_comments(_function_body(src, "function onXRFrame"))
    for forbidden in ("blend", "passthrough", "immersive"):
        assert forbidden not in body, (
            "%r reaches the per-frame message -- the protocol has learned "
            "about the display" % forbidden)
