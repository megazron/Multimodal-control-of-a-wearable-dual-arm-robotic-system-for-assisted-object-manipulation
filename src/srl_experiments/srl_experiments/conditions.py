#!/usr/bin/env python3
"""
conditions.py — counterbalancing, and the ordering guarantees it must give.

Fatigue on this rig grows monotonically with time on task (the master arm has
no gravity compensation — see docs/system/05_gravity_and_load.md), and so does
condition order. Counterbalancing does not remove that; it converts a BIAS
into VARIANCE, which is the difference between a wrong answer and a noisy one.

A balanced Latin square is used rather than a plain one because a plain Latin
square balances first-order position but NOT immediate sequence: with a plain
square, condition B may follow A far more often than A follows B, and any
carry-over from A then loads onto B. Williams' construction balances both.
"""
import itertools
import random


def williams_square(n):
    """Williams design: every condition appears once per position AND every
    ordered pair (a immediately before b) appears equally often.

    For even n one square suffices; for odd n the square and its reverse are
    both needed, giving 2n sequences.
    """
    if n < 2:
        return [[0]]
    rows = []
    for i in range(n):
        row = []
        for j in range(n):
            if j % 2 == 0:
                k = j // 2
            else:
                k = n - 1 - (j - 1) // 2
            row.append((i + k) % n)
        rows.append(row)
    if n % 2 == 1:
        rows += [list(reversed(r)) for r in rows]
    return rows


def assign_order(conditions, participant_index):
    """The condition order for one participant. Deterministic in the index, so
    a session can be reconstructed from the manifest alone."""
    sq = williams_square(len(conditions))
    return [conditions[i] for i in sq[participant_index % len(sq)]]


def check_balance(conditions, n_participants):
    """Report how well the design is actually balanced at this n.

    Reported rather than assumed: with n_participants not a multiple of the
    square size, the balance is only approximate, and knowing by how much is
    the difference between a caveat and a surprise.
    """
    n = len(conditions)
    sq = williams_square(n)
    orders = [assign_order(conditions, i) for i in range(n_participants)]
    pos = {c: [0] * n for c in conditions}
    pair = {(a, b): 0 for a, b in itertools.permutations(conditions, 2)}
    for o in orders:
        for p, c in enumerate(o):
            pos[c][p] += 1
        for a, b in zip(o, o[1:]):
            pair[(a, b)] += 1
    pos_spread = max(max(v) - min(v) for v in pos.values())
    pair_spread = (max(pair.values()) - min(pair.values())) if pair else 0
    return dict(n_participants=n_participants, square_size=len(sq),
                position_counts={c: v for c, v in pos.items()},
                position_imbalance=pos_spread,
                sequence_imbalance=pair_spread,
                perfectly_balanced=(n_participants % len(sq) == 0))


def block_trials(scenarios, repeats, seed=0):
    """Trial order inside a block: repeats of each scenario, shuffled, but
    never the same scenario twice in a row where that can be avoided."""
    rng = random.Random(seed)
    pool = [s for s in scenarios for _ in range(repeats)]
    for _ in range(200):
        rng.shuffle(pool)
        if all(a != b for a, b in zip(pool, pool[1:])):
            break
    return pool
