"""The map may add an obstacle; it may not remove one. Pinned, not argued.

`map_obstacles_node` is the first seam from measured objects to MoveIt, and
its one safety property is the direction: an object that vanishes from view
STAYS in the planning scene until a person triggers the clear. "The camera
stopped seeing it" and "it is gone" are different claims, and a planner that
believes the first as the second plans through furniture.

Also pinned: the namespace (mapped_* only -- two clients writing one planning
scene cost this project a day), the diff (only changes are published, never a
stream of identical scenes), the wearer exclusion (a wrist camera that sees
the person must not file them as furniture), and the size floor. All against
constructed boxes -- arithmetic ground truth.
"""
import pytest

pytest.importorskip("rclpy")

from srl_perception import map_obstacles_node as MO  # noqa: E402


def obs(centre, extents=(0.04, 0.04, 0.04)):
    return dict(centre=list(centre), extents_m=list(extents))


def store(**kw):
    kw.setdefault("merge_m", 0.06)
    kw.setdefault("min_extent_m", 0.015)
    return MO.MappedStore(**kw)


# On the table, well clear of the wearer's volume (y >= 0.43 is the near edge).
A = (0.40, 0.50, 1.12)
B = (0.10, 0.50, 1.12)


def test_every_id_is_in_the_mapped_namespace():
    s = store()
    s.update([obs(A), obs(B)])
    assert s.objs and all(i.startswith("mapped_") for i in s.objs)


def test_a_vanished_object_is_not_removed():
    """The property the node exists for. Cycles with the object absent change
    nothing; only the explicit clear does."""
    s = store()
    s.update([obs(A), obs(B)])
    n = len(s.objs)
    for _ in range(5):
        s.update([obs(B)])              # A has vanished from view
    assert len(s.objs) == n, "a vanished object was silently dropped"


def test_a_smaller_observation_never_shrinks_the_box():
    s = store()
    s.update([obs(A, (0.10, 0.10, 0.10))])
    (lo0, hi0), = [tuple(map(tuple, v)) for v in s.objs.values()]
    s.update([obs(A, (0.03, 0.03, 0.03))])
    (lo1, hi1), = [tuple(map(tuple, v)) for v in s.objs.values()]
    assert lo1 == lo0 and hi1 == hi0, "the box shrank"


def test_a_larger_observation_grows_the_box():
    s = store()
    s.update([obs(A, (0.04, 0.04, 0.04))])
    res = s.update([obs(A, (0.10, 0.10, 0.10))])
    assert res["grown"] and not res["added"]


def test_the_diff_publishes_only_changes():
    s = store()
    s.update([obs(A)])
    d1 = s.diff()
    assert len(d1) == 1
    s.mark_published([e[0] for e in d1])
    assert s.diff() == [], "an unchanged store re-published"
    s.update([obs(A)])                   # identical observation
    assert s.diff() == [], "an identical observation re-published"
    s.update([obs(B)])
    assert len(s.diff()) == 1, "only the NEW object should be in the diff"


def test_clear_is_explicit_and_returns_what_it_forgot():
    s = store()
    s.update([obs(A), obs(B)])
    ids = s.clear()
    assert sorted(ids) == sorted(["mapped_0", "mapped_1"])
    assert s.objs == {}


def test_the_wearer_volume_is_excluded_by_name():
    s = store()
    inside = (0.0, 0.0, 1.20)            # the mannequin's chest
    res = s.update([obs(inside, (0.30, 0.20, 0.40))])
    assert res["added"] == [] and s.objs == {}
    assert res["excluded"], "the person was filed as furniture"
    assert "wearer" in res["excluded"][0]["why"]


def test_an_object_under_the_size_floor_is_excluded_by_name():
    s = store()
    res = s.update([obs(A, (0.005, 0.005, 0.005))])
    assert s.objs == {}
    assert "floor" in res["excluded"][0]["why"]


def test_a_clean_object_is_accepted_so_the_gates_are_not_a_wall():
    """A gate that rejects everything is not a gate."""
    s = store()
    res = s.update([obs(A)])
    assert res["added"] == ["mapped_0"] and res["excluded"] == []


def test_the_footprint_is_circumscribed_never_understated():
    """No yaw is carried, so the horizontal box must take the LONGEST extent
    in both x and y -- too small is a collision, too big is a detour."""
    lo, hi = MO.observation_box((0.0, 0.0, 1.0), (0.12, 0.04, 0.06))
    assert hi[0] - lo[0] == pytest.approx(0.12)
    assert hi[1] - lo[1] == pytest.approx(0.12)
    assert hi[2] - lo[2] == pytest.approx(0.06)


def test_world_map_loading_refuses_an_empty_or_missing_file(tmp_path):
    with pytest.raises(FileNotFoundError):
        MO.load_world_map(str(tmp_path / "nowhere.json"))
    p = tmp_path / "empty.json"
    p.write_text('{"objects": []}')
    with pytest.raises(ValueError):
        MO.load_world_map(str(p))


def test_world_map_loading_reads_the_saved_shape(tmp_path):
    """The shape `world_model.save()` writes: objects with centre/extents."""
    p = tmp_path / "map.json"
    p.write_text('{"objects": [{"centre": [0.4, 0.5, 1.12], '
                 '"extents": [0.04, 0.04, 0.04]}]}')
    out, note = MO.load_world_map(str(p))
    assert out == [dict(centre=[0.4, 0.5, 1.12],
                        extents_m=[0.04, 0.04, 0.04])]
    assert "1 object(s)" in note


def test_collision_objects_carry_only_mapped_ids_and_add_operations():
    moveit_msgs = pytest.importorskip("moveit_msgs.msg")
    s = store()
    s.update([obs(A), obs(B)])
    cos = MO.collision_objects_for(s.diff())
    assert len(cos) == 2
    for co in cos:
        assert co.id.startswith("mapped_")
        assert co.operation == moveit_msgs.CollisionObject.ADD
        assert co.header.frame_id == "world"
        assert co.primitives[0].dimensions == pytest.approx([0.04, 0.04, 0.04])


def test_remove_messages_exist_only_on_the_clear_path():
    moveit_msgs = pytest.importorskip("moveit_msgs.msg")
    s = store()
    s.update([obs(A)])
    ids = s.clear()
    cos = MO.collision_objects_for([(i, (0, 0, 0), (0, 0, 0)) for i in ids],
                                   remove=True)
    assert all(co.operation == moveit_msgs.CollisionObject.REMOVE
               for co in cos)
    assert all(not co.primitives for co in cos)
