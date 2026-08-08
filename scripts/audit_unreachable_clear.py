#!/usr/bin/env python3
"""Audit for the LATCH pattern, not for its instances.

    bash scripts/audit_unreachable_clear.py          # or python3

THE PATTERN. A piece of state is SET in one branch and CLEARED only on a path
that cannot be reached while the state is set. The state then latches, and
because the thing holding it is usually a guard or a health flag, the symptom
is silence: the system reports a permanent fault, or reports nothing at all,
and nothing in the logs says why.

It has now occurred five times in this project, the fifth inside the code
written to catch the first four (`ik_inflight`, cleared on a line that only
runs when the in-flight flag is already False). Grepping for each new instance
after it bites does not scale, so this looks for the shape.

Three checks, all deliberately syntactic -- this is a smell detector, and
every hit is meant to be read, not obeyed:

  1. BLOCK-THEN-RETURN. `x.block(N)` followed by `return` in the same branch,
     where the only `x.clear(N)` sits AFTER that return in the same function.
     The clear runs only when the block did not fire.

  2. GUARDED SELF-CLEAR. `if <flag>: ... return` where the assignment
     `<flag> = False` appears only inside a branch guarded by `not <flag>`,
     or after a return that the flag itself triggers.

  3. SET-WITHOUT-CLEAR. An attribute assigned a truthy latch value somewhere
     in a class and never assigned a falsy one anywhere in that class.

Exit code is 0 always: this is a report, not a gate. A finding is a question
("can this clear ever run while the state is set?"), not a verdict.
"""
import ast
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent / 'src'
SKIP = {'build', 'install', 'log', '.git', 'test'}


def branch_returns(node):
    """Does this if-branch end in a return/continue/break?"""
    for st in ast.walk(node):
        if isinstance(st, (ast.Return, ast.Continue, ast.Break)):
            return True
    return False


def check_block_then_return(tree, path, out):
    """Check 1: block(N) + return, with clear(N) only later in the function."""
    for fn in [n for n in ast.walk(tree)
               if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))]:
        blocked, cleared = {}, {}
        for n in ast.walk(fn):
            if not (isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)):
                continue
            if not n.args or not isinstance(n.args[0], ast.Constant):
                continue
            name = n.args[0].value
            if not isinstance(name, str):
                continue
            if n.func.attr == 'block':
                blocked.setdefault(name, []).append(n.lineno)
            elif n.func.attr == 'clear':
                cleared.setdefault(name, []).append(n.lineno)
        for name, blines in blocked.items():
            clines = cleared.get(name)
            if not clines:
                continue
            for st in ast.walk(fn):
                if not isinstance(st, ast.If) or not branch_returns(st):
                    continue
                inner = [c.lineno for c in ast.walk(st)
                         if isinstance(c, ast.Call)
                         and isinstance(c.func, ast.Attribute)
                         and c.func.attr == 'block'
                         and c.args and isinstance(c.args[0], ast.Constant)
                         and c.args[0].value == name]
                if not (inner and all(cl > max(inner) for cl in clines)):
                    continue
                # The distinguishing question, and the whole point of this
                # audit: can the GUARD become false while the block is held?
                # If every assignment to the state in the guard sits BELOW the
                # return, in this same function, then holding the block
                # prevents the very assignment that would release it, and the
                # block latches forever. If the guard is mutated elsewhere -- a
                # different callback, a timer, a parameter -- the next call
                # re-evaluates it and reaches the clear normally.
                guard = {n.attr for n in ast.walk(st.test)
                         if isinstance(n, ast.Attribute)
                         and isinstance(n.value, ast.Name) and n.value.id == 'self'}
                if not guard:
                    continue
                elsewhere, below = set(), set()
                for a in ast.walk(tree):
                    if not isinstance(a, ast.Assign):
                        continue
                    for t in a.targets:
                        if (isinstance(t, ast.Attribute)
                                and isinstance(t.value, ast.Name)
                                and t.value.id == 'self' and t.attr in guard):
                            if fn.lineno <= a.lineno <= max(clines) and \
                                    a.lineno > max(inner):
                                below.add(t.attr)
                            elif not (fn.lineno <= a.lineno <= max(clines)):
                                elsewhere.add(t.attr)
                risky = guard - elsewhere
                sev = 'LATCH-RISK' if risky & (below or risky) and not elsewhere \
                    else 'review'
                if sev == 'LATCH-RISK':
                    why = ('released only by assignments BELOW the return -- '
                           'the block prevents its own clear')
                else:
                    why = ('also assigned outside this function (%s), so a '
                           'later call re-evaluates and clears'
                           % '/'.join(sorted(elsewhere)))
                msg = ("'%s' blocked then returns; only clear() is line %s, "
                       "below it. Guard reads %s; %s"
                       % (name, ','.join(map(str, clines)),
                          '/'.join(sorted(guard)), why))
                out.append((path, min(inner), 'block-then-return/' + sev, msg))
                break


def check_set_without_clear(tree, path, out):
    """Check 3: self.X set truthy in a class and never set falsy."""
    for cls in [n for n in ast.walk(tree) if isinstance(n, ast.ClassDef)]:
        truthy, falsy = {}, set()
        for n in ast.walk(cls):
            if not isinstance(n, ast.Assign):
                continue
            for t in n.targets:
                if not (isinstance(t, ast.Attribute)
                        and isinstance(t.value, ast.Name)
                        and t.value.id == 'self'):
                    continue
                v = n.value
                if isinstance(v, ast.Constant) and v.value is True:
                    truthy.setdefault(t.attr, n.lineno)
                elif isinstance(v, ast.Constant) and v.value in (False, None):
                    falsy.add(t.attr)
                else:
                    falsy.add(t.attr)          # unknown value: assume clearable
        for attr, line in truthy.items():
            if attr not in falsy:
                out.append((path, line, 'set-without-clear',
                            "self.%s is set True and never assigned a falsy "
                            "value anywhere in class %s" % (attr, cls.name)))


def main():
    findings = []
    files = []
    for dirpath, dirnames, filenames in os.walk(ROOT):
        dirnames[:] = [d for d in dirnames if d not in SKIP]
        for f in filenames:
            if f.endswith('.py'):
                files.append(Path(dirpath) / f)
    for p in sorted(files):
        try:
            tree = ast.parse(p.read_text())
        except SyntaxError:
            continue
        rel = str(p.relative_to(ROOT.parent))
        check_block_then_return(tree, rel, findings)
        check_set_without_clear(tree, rel, findings)

    print('LATCH-PATTERN AUDIT — %d python files under src/' % len(files))
    print('Looking for state set in one branch and cleared only where it '
          'cannot be reached.\n')
    if not findings:
        print('  no occurrences found')
    by_kind = {}
    for f in findings:
        by_kind.setdefault(f[2], []).append(f)
    for kind in sorted(by_kind):
        print('%s  (%d)' % (kind.upper(), len(by_kind[kind])))
        for path, line, _, msg in sorted(by_kind[kind]):
            print('  %s:%d  %s' % (path, line, msg))
        print()
    return 0


if __name__ == '__main__':
    sys.exit(main())
