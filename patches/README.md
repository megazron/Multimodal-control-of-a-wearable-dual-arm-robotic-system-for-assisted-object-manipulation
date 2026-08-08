# Vendor patches — REQUIRED for two real arms

`src/ros2_kortex/`, `src/ros2_robotiq_gripper/` and `src/ros2_kortex_vision/`
are **gitignored**. Every fix below lives only in your working tree and is
**lost on any vendor update, re-clone or `rosdep`/vcs refresh** — which is why
they are captured here as patches rather than described in prose.

```bash
# after re-fetching the vendor sources
cd ~/kortex_ws
for p in patches/*.patch; do patch -p1 < "$p"; done
colcon build --packages-select kortex_driver kortex_description robotiq_description
python3 scripts/audit_dual_arm_collisions.py     # must print PASS
```

To check whether they are currently applied:

```bash
for p in patches/*.patch; do
  patch -p1 --dry-run --reverse --force < "$p" >/dev/null 2>&1 \
    && echo "applied:     $p" || echo "NOT applied: $p"
done
```

## The bug class these all belong to

**ros2_control resource names must be unique across the whole robot.** The
vendor macros are written for ONE arm and hardcode several names. Instantiate
a macro twice and both hardware components register the same resource; the
second to load exports **nothing**.

The symptom appears nowhere near the cause: an interface reading
`[unavailable] [unclaimed]` while its twin on the other arm reads
`[available] [claimed]`, and a controller that refuses to activate with
"Failed to activate controller". Nothing says "name collision".

Five instances have been found so far, each costing a debugging session:

| # | resource | where | fixed by |
| --- | --- | --- | --- |
| 1 | gripper hardware component **name** | xacro | `${prefix}` in the macro (pre-existing) |
| 2 | `reactivate_gripper` GPIO **declaration** | xacro | 0003 |
| 3 | gripper `COM_port`, both defaulting to `/dev/ttyUSB0` | xacro | per-arm `left/right_gripper_com_port` args (pre-existing) |
| 4 | `tcp/twist.*` and `reset_fault/*` | **driver C++** | 0001 + 0002 |
| 5 | `reactivate_gripper` GPIO **export** | **driver C++** | 0003 |

`scripts/audit_dual_arm_collisions.py` now checks for all five shapes
statically, including the C++ ones, so the sixth should be found by the tool
rather than by a lost afternoon.

---

## 0001 — kortex_driver: prefix the non-joint resources

`KortexMultiInterfaceHardware` exported three resources under **hardcoded
string literals**, invisible to any URDF-level check:

- `tcp` — six twist command interfaces (`twist.linear.{x,y,z}`,
  `twist.angular.{x,y,z}`)
- `reset_fault` — state interface `internal_fault`
- `reset_fault` — command interfaces `command` and `async_success`

The joints were already prefixed by the xacro; these were not.

**The fix** reads a new `prefix` hardware parameter into `resource_prefix_` and
composes it into those names, in `export_state_interfaces()`,
`export_command_interfaces()`, and the four key comparisons inside
`prepare_command_mode_switch()` / `perform_command_mode_switch()` — miss the
comparisons and the interfaces are exported correctly but never matched.

`prefix` defaults to empty, so **single-arm behaviour is bit-identical to
upstream**. Literal quotes are stripped from the value, because the vendor
xacro writes `tf_prefix` wrapped in `"` and the same accident here would
export a resource literally named `left_`.

> `tf_prefix` already existed as a hardware parameter and looked like the
> natural place for this. It is **declared in the xacro and never read by the
> driver** — dead. Do not "reuse" it without checking that.

## 0002 — kortex_description: pass `prefix` into `<hardware>`

Supplies the parameter 0001 reads. Applied to all three arm macros (gen3
7-DOF, gen3 6-DOF, gen3_lite 6-DOF) so a future arm swap does not silently
lose it.

## 0003 — robotiq: prefix `reactivate_gripper`, in BOTH places

The xacro half of this was applied in an earlier session and verified — but
**only against `mock_components/GenericSystem`**, which reads the URDF and
never runs the vendor C++. `robotiq_driver` carries a `COLCON_IGNORE` (it
needs a `serial` CMake package unavailable here), so it **has never been
built or run in this workspace**.

`robotiq_driver/src/hardware_interface.cpp` still exported the GPIO under the
literal name `reactivate_gripper`. So after the xacro fix the URDF declared
`left_reactivate_gripper` while the driver exported `reactivate_gripper` —
they disagreed, and on real hardware that is a fresh failure, not a fix.

This patch prefixes **both** the declaration and the C++ export.

**UNVERIFIED — it cannot be compiled here.** `robotiq_driver` will not build
until the `serial` dependency is resolved; see
`src/ros2_robotiq_gripper/README_BUILD.md`. Treat 0003's C++ half as
*written and reviewed but never executed*. The xacro half is exercised by
every sim run.

---

## What is verified, and what is not

| patch | built | exercised |
| --- | --- | --- |
| 0001 | **yes** — `colcon build --packages-select kortex_driver` | never against a real arm |
| 0002 | yes — the URDF renders and the audit passes | never against a real arm |
| 0003 xacro | yes | every sim run |
| 0003 C++ | **no — package is COLCON_IGNOREd** | never |

**No patch here has run against real hardware.** They remove a class of
collision that is provable statically; whether two real Kortex sessions and
two real grippers then come up is `docs/NEXT_SESSION.md` item 2 and item 4.

## The duplicate that came FIRST: `robotiq_2f_85_macro`

Before the GPIO fix there was a plainer collision. Both grippers instantiate
`robotiq_2f_85_macro`, which declared its `<hardware>` component with a fixed
name. Two instantiations therefore registered **the same hardware component
name**, and the second to load lost.

That one was fixed upstream-style by prefixing the hardware *name* — and that
fix is exactly what made the next bug so hard to see. It prefixed the
component but **not** the `<gpio>` inside it, so the URDF then declared
`left_reactivate_gripper` while the driver still exported a bare
`reactivate_gripper` C++ literal. The two disagreed silently, and because
`robotiq_driver` is `COLCON_IGNORE`d and every gripper test to date ran
against `mock_components/GenericSystem` — which reads the URDF and never runs
that C++ — nothing caught it.

**The lesson worth keeping:** a partial fix to a duplicate-resource bug is
worse than none, because it removes the loud symptom and leaves the quiet
one. `scripts/audit_dual_arm_collisions.py` now checks all five shapes,
including C++ string literals, which no URDF-level check can see.

