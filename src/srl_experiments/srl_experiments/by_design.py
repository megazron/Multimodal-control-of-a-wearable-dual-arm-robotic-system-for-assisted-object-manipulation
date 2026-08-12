"""ABSENCE BY DESIGN vs A REAL GAP -- one definition, not one per reporter.

The distinction appeared twice in one hour, in two separate files, and both
times the reporter rendered a deliberate design decision identically to a
defect:

  * status_table.py showed T2/T3's empty VR cells as gaps. They are not: the
    session plan runs T2 in the two anchors only. Someone reading it would
    have gone hunting a data path that was not broken -- and that path HAD
    broken once before, so the false alarm was entirely credible.
  * verify_scan_view.py FAILED the right camera for seeing no objects. T1 is
    a left-arm task; all four cubes sit at positive x. The pose was fine.

Both were fixed separately, which is how the second one happened. The shape
is always the same and belongs in one place: something is absent; is it
absent because nobody asked for it, or because something is broken?

A reporter that cannot tell the difference is worse than one that omits the
column, because a green row nobody re-checks and a red row nobody believes
fail in the same direction -- they both stop carrying information.
"""


class Expectation:
    """What SHOULD exist, so what is missing can be classified.

    Usage: build one from the plan, then ask it about each observed key.

        exp = Expectation(planned_cells, label="session plan")
        exp.classify(("t2", "02_vr_teleop"), present=False)
        -> ("by_design", "outside the session plan")
    """

    BY_DESIGN = "by_design"
    GAP = "gap"
    PRESENT = "present"
    UNEXPECTED = "unexpected"

    def __init__(self, expected, label="the plan", reason=None):
        self.expected = set(expected)
        self.label = label
        self.reason = reason or ("outside %s -- absence is BY DESIGN"
                                 % label)

    def classify(self, key, present):
        """One of PRESENT / GAP / BY_DESIGN / UNEXPECTED, with a reason."""
        want = key in self.expected
        if present and want:
            return (self.PRESENT, "")
        if present and not want:
            # NOT an error, but worth naming: something ran that the plan
            # does not account for, and analysis must know it exists.
            return (self.UNEXPECTED, "present but %s does not ask for it"
                    % self.label)
        if want:
            return (self.GAP, "%s asks for it and it is MISSING" % self.label)
        return (self.BY_DESIGN, self.reason)

    def gaps(self, observed):
        """Only the REAL gaps -- expected and absent."""
        return sorted(k for k in self.expected if k not in set(observed))

    def summary(self, observed):
        obs = set(observed)
        g = self.gaps(obs)
        return dict(expected=len(self.expected), present=len(obs & self.expected),
                    gaps=g, unexpected=sorted(obs - self.expected))
