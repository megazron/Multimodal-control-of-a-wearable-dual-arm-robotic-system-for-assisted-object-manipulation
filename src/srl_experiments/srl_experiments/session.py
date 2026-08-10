#!/usr/bin/env python3
"""Participant session state: the protocol, and surviving a failure mid-trial.

WRITTEN TO DISK AFTER EVERY TRIAL, because the alternative is telling a
participant who has already given an hour that we are starting again. A crash
must RESUME, not restart, and that means the file has to be correct after an
abrupt kill -- so every write is atomic (tmp + os.replace), never a partial
rewrite of the live file.

INVALID IS NOT LOST. A trial that fails mid-flight is recorded with its cause
and kept. Deleting it would make the session look cleaner than it was, and the
analysis excludes on the `valid` column anyway (srl_experiments.validity is
the single definition, shared so the experiments cannot drift).

THE THREE RECOVERIES ARE ONE CLICK EACH, and they are the whole design:
REDO re-runs the same trial, SKIP moves on leaving the failure recorded, ABORT
ends the session with the data kept and flagged. Nobody should have to
diagnose anything with a participant in the room.
"""
import json
import os
import time
import uuid

# States a session can be in.
IDLE, CONSENT, FAMILIAR, RUNNING, PAUSED, DONE, ABORTED = (
    "idle", "consent", "familiarisation", "running", "paused", "done",
    "aborted")

CONSENT_STEPS = (
    "information sheet given and read",
    "opportunity to ask questions",
    "consent to participate",
    "consent to data being recorded",
    "right to withdraw at any time, without reason, explained",
    "emergency stop shown to the participant and tested by them",
)


def _sessions_dir():
    return os.environ.get("SRL_SESSION_DIR") or os.path.join(
        os.path.expanduser("~"), ".srl_sessions")


class Trial:
    __slots__ = ("index", "block", "mode", "task", "started", "ended",
                 "valid", "cause", "metrics", "attempt")

    def __init__(self, index, block, mode, task, attempt=1):
        self.index = index
        self.block = block
        self.mode = mode
        self.task = task
        self.attempt = attempt
        self.started = None
        self.ended = None
        self.valid = None          # None = not finished; True/False after
        self.cause = ""
        self.metrics = {}

    def as_dict(self):
        return {k: getattr(self, k) for k in self.__slots__}

    @staticmethod
    def from_dict(d):
        t = Trial(d["index"], d["block"], d["mode"], d["task"],
                  d.get("attempt", 1))
        for k in ("started", "ended", "valid", "cause", "metrics"):
            setattr(t, k, d.get(k))
        return t


class Session:
    """One participant's session. Owns the plan, the trials and the file."""

    VERSION = 3

    def __init__(self, participant, plan, sid=None, path=None):
        # NO IDENTIFYING DATA. srl_experiments.trial_logger already raises on
        # name/email/dob/address/phone; the participant id here is a code.
        for bad in ("name", "email", "dob", "address", "phone"):
            if bad in str(participant).lower():
                raise ValueError(
                    "participant id looks like identifying data (%r). Use a "
                    "code." % participant)
        self.participant = str(participant)
        self.sid = sid or uuid.uuid4().hex[:12]
        self.plan = list(plan)             # [(block, mode, task), ...]
        self.trials = []
        self.state = IDLE
        self.consent = {s: False for s in CONSENT_STEPS}
        self.started = None
        self.ended = None
        self.abort_reason = ""
        self.log = []                      # readable DURING the session
        self.path = path or os.path.join(_sessions_dir(),
                                         "session_%s.json" % self.sid)

    # ------------------------------------------------------------- logging
    def note(self, text, level="info"):
        """A line the experimenter can read WHILE it is happening."""
        entry = dict(t=time.time(), level=level, text=str(text))
        self.log.append(entry)
        return entry

    # -------------------------------------------------------------- progress
    @property
    def done_count(self):
        return sum(1 for t in self.trials if t.ended is not None)

    @property
    def current(self):
        for t in self.trials:
            if t.ended is None:
                return t
        return None

    def next_index(self):
        """Index into the plan of the next trial to run.

        An index is CONSUMED when it completed validly, or when the operator
        SKIPPED it. The first version only consumed valid completions, so
        SKIP left the index open and begin_trial handed back the same trial
        for ever -- the button said "skip and continue" and did neither.
        Caught by the break test, which is what it is for.
        """
        done = {t.index for t in self.trials
                if t.ended is not None and t.valid is not False}
        done |= set(getattr(self, "_skipped", set()))
        for i in range(len(self.plan)):
            if i not in done:
                return i
        return None

    # ---------------------------------------------------------------- flow
    def give_consent(self, step, value=True):
        if step not in self.consent:
            raise KeyError(step)
        self.consent[step] = bool(value)
        self.note("consent: %s = %s" % (step, self.consent[step]))
        self.save()

    @property
    def consent_complete(self):
        return all(self.consent.values())

    def start(self):
        """Begin the session. REFUSES without complete consent."""
        if not self.consent_complete:
            missing = [s for s, v in self.consent.items() if not v]
            raise RuntimeError("consent incomplete: %s" % "; ".join(missing))
        self.state = RUNNING
        self.started = self.started or time.time()
        self.note("session started", "good")
        self.save()

    def begin_trial(self):
        i = self.next_index()
        if i is None:
            self.state = DONE
            self.ended = time.time()
            self.note("all trials complete", "good")
            self.save()
            return None
        block, mode, task = self.plan[i]
        prior = [t for t in self.trials if t.index == i]
        t = Trial(i, block, mode, task, attempt=len(prior) + 1)
        t.started = time.time()
        self.trials.append(t)
        self.note("trial %d/%d begins — block %s, mode %s, task %s (attempt %d)"
                  % (i + 1, len(self.plan), block, mode, task, t.attempt))
        self.save()
        return t

    def end_trial(self, valid=True, cause="", metrics=None):
        t = self.current
        if t is None:
            return None
        t.ended = time.time()
        t.valid = bool(valid)
        t.cause = cause
        t.metrics = metrics or {}
        self.note("trial %d ended: %s%s"
                  % (t.index + 1, "VALID" if valid else "INVALID",
                     (" — " + cause) if cause else ""),
                  "good" if valid else "bad")
        self.save()                       # AFTER EVERY TRIAL, without fail
        return t

    def fail_trial(self, cause):
        """Something broke mid-trial. Record it, keep it, name it."""
        return self.end_trial(valid=False, cause=cause)

    # ------------------------------------------------------- the three keys
    def redo(self):
        """Run the same plan index again. The failed attempt is KEPT."""
        t = self.trials[-1] if self.trials else None
        if t is not None and t.ended is None:
            self.fail_trial("redo requested before the trial ended")
        self.note("REDO requested", "warn")
        self.save()

    def skip(self):
        """Move past this plan index, leaving the failure on the record."""
        t = self.trials[-1] if self.trials else None
        if t is not None and t.ended is None:
            self.fail_trial("skipped by operator")
        elif t is not None:
            # mark the index consumed even though it was invalid
            t.cause = (t.cause + "; skipped").strip("; ")
        self._skipped = getattr(self, "_skipped", set())
        if t is not None:
            self._skipped.add(t.index)
        self.note("SKIP — continuing to the next trial", "warn")
        self.save()

    def abort(self, reason, by="operator"):
        """Stop everything. Data is KEPT and flagged."""
        t = self.current
        if t is not None:
            self.fail_trial("session aborted: %s" % reason)
        self.state = ABORTED
        self.ended = time.time()
        self.abort_reason = "%s (%s)" % (reason, by)
        self.note("SESSION ABORTED by %s — %s" % (by, reason), "bad")
        self.save()

    # ---------------------------------------------------------------- disk
    def as_dict(self):
        return dict(version=self.VERSION, sid=self.sid,
                    participant=self.participant, state=self.state,
                    plan=[list(p) for p in self.plan],
                    consent=self.consent, started=self.started,
                    ended=self.ended, abort_reason=self.abort_reason,
                    skipped=sorted(getattr(self, "_skipped", set())),
                    trials=[t.as_dict() for t in self.trials],
                    log=self.log[-500:])

    def save(self):
        """ATOMIC. A crash mid-write must not leave a half-written session."""
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        tmp = self.path + ".tmp"
        with open(tmp, "w") as fh:
            json.dump(self.as_dict(), fh, indent=1)
            fh.flush()
            os.fsync(fh.fileno())
        os.replace(tmp, self.path)
        return self.path

    @staticmethod
    def load(path):
        d = json.load(open(path))
        if d.get("version") != Session.VERSION:
            raise ValueError("session file version %r, expected %d — refusing "
                             "to resume a schema this code does not know"
                             % (d.get("version"), Session.VERSION))
        s = Session(d["participant"], [tuple(p) for p in d["plan"]],
                    sid=d["sid"], path=path)
        s.state = d["state"]
        s.consent = d.get("consent", s.consent)
        s.started = d.get("started")
        s.ended = d.get("ended")
        s.abort_reason = d.get("abort_reason", "")
        s._skipped = set(d.get("skipped", []))
        s.trials = [Trial.from_dict(t) for t in d.get("trials", [])]
        s.log = d.get("log", [])
        return s

    @staticmethod
    def resumable(directory=None):
        """Sessions that were interrupted, newest first."""
        d = directory or _sessions_dir()
        out = []
        if not os.path.isdir(d):
            return out
        for fn in os.listdir(d):
            # ANY json that parses as a session, not just the default name.
            # Matching on the "session_" prefix meant a session written to an
            # explicit path was invisible to resume -- the one case where the
            # operator most likely chose the path deliberately.
            if not fn.endswith(".json"):
                continue
            p = os.path.join(d, fn)
            try:
                s = Session.load(p)
            except Exception:                                    # noqa: BLE001
                continue
            if s.state in (RUNNING, PAUSED, CONSENT, FAMILIAR):
                out.append(s)
        out.sort(key=lambda s: s.started or 0, reverse=True)
        return out
