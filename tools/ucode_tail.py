#!/usr/bin/env python3
"""Per-chord residual tail probabilities out to 1.7 half (docs/246 step 1).

Same quantiser, same chords, same "step measured from the alphabet" as
tools/ucode_margin.py (docs/244 step 1).  The new quantity is the whole tail
of |e| in units of the chord's own half-step, so that the trellis's coded
boundary can be read off it as a probability instead of as a support bound.

    P_c(t) = #{symbols in chord c with |e| > t * step_c/2} / n_c
"""
import sys, array, math
from collections import defaultdict

UCODES = range(256)
def dec(u):
    b = u ^ 0xFF
    e = (b >> 4) & 7
    m = b & 0xF
    mag = (((m << 3) + 0x84) << e) - 0x84
    return -mag if (b & 0x80) else mag

LEVELS = [(abs(dec(u)), u) for u in UCODES]
LEVELS.sort()
MAGS = [m for m, _ in LEVELS]

def enc(x):
    a = min(abs(int(x)), 32124)
    lo, hi = 0, len(MAGS) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if MAGS[mid] < a: lo = mid + 1
        else: hi = mid
    cand = [lo]
    if lo > 0: cand.append(lo - 1)
    best = min(cand, key=lambda i: abs(MAGS[i] - a))
    lvl = LEVELS[best][0]
    return (-lvl if x < 0 else lvl), (LEVELS[best][1] & 0x7F) >> 4

_TAB = {}
for _u in UCODES:
    _c = (_u & 0x7F) >> 4
    _TAB.setdefault(_c, []).append(abs(dec(_u)))
STEP = {}
for _c, _v in _TAB.items():
    _v = sorted(set(_v))
    STEP[_c] = min(b - a for a, b in zip(_v, _v[1:]))

def step_of(c):
    return STEP[c]

def load(path):
    a = array.array('h')
    with open(path, 'rb') as f:
        a.frombytes(f.read())
    if sys.byteorder == 'big':
        a.byteswap()
    return a

TS = [round(1.0 + 0.02 * i, 2) for i in range(36)]  # 1.00 .. 1.70

def tails(xs, lo, hi, label):
    # per chord: list of |e| in units of that chord's half-step
    per = defaultdict(list)
    for k in range(lo, hi):
        x = xs[k]
        if x == 0:
            continue
        q, c = enc(x)
        per[c].append(abs(x - q) / (step_of(c) / 2.0))
    print(f"\n### {label}  samples {lo}..{hi} (={hi-lo})")
    hdr = "chord step    n "
    for t in (1.00, 1.20, 1.40, 1.44, 1.46, 1.48, 1.50, 1.55, 1.60, 1.62, 1.66, 1.70):
        hdr += f" >{t:4.2f}"
    print(hdr)
    rows = {}
    for c in sorted(per):
        v = per[c]
        n = len(v)
        v.sort()
        line = f"{c:5d} {step_of(c):5d} {n:6d} "
        rows[c] = (n, v)
        for t in (1.00, 1.20, 1.40, 1.44, 1.46, 1.48, 1.50, 1.55, 1.60, 1.62, 1.66, 1.70):
            import bisect
            k = n - bisect.bisect_right(v, t)
            line += f" {k:5d}" if k else "     ."
        print(line)
    # weighted totals
    tot = sum(n for n, _ in rows.values())
    line = f"{'ALL':>5} {'':>5} {tot:6d} "
    for t in (1.00, 1.20, 1.40, 1.44, 1.46, 1.48, 1.50, 1.55, 1.60, 1.62, 1.66, 1.70):
        s = 0
        for c, (n, v) in rows.items():
            import bisect
            s += n - bisect.bisect_right(v, t)
        line += f" {s:5d}" if s else "     ."
    print(line)
    return rows

def fine(rows, label):
    print(f"\n--- {label}: P(|e| > t half), per chord, %.6f%%")
    hdr = "chord    n  "
    for t in (1.40, 1.44, 1.46, 1.50, 1.55, 1.62, 1.70):
        hdr += f"  t={t:4.2f}"
    print(hdr)
    import bisect
    for c in sorted(rows):
        n, v = rows[c]
        line = f"{c:5d} {n:5d}  "
        for t in (1.40, 1.44, 1.46, 1.50, 1.55, 1.62, 1.70):
            k = n - bisect.bisect_right(v, t)
            line += f" {100.0*k/n:8.5f}" if k else "        ."
        print(line)
    tot = sum(n for n, _ in rows.values())
    line = f"{'ALL':>5} {tot:5d}  "
    for t in (1.40, 1.44, 1.46, 1.50, 1.55, 1.62, 1.70):
        s = sum(n - bisect.bisect_right(v, t) for n, v in rows.values())
        line += f" {100.0*s/tot:8.5f}" if s else "        ."
    print(line)

if __name__ == '__main__':
    path = sys.argv[1]
    xs = load(path)
    print(f"{path}: {len(xs)} samples, {len(set(abs(x) for x in xs))} distinct |x|")
    lo = int(sys.argv[2]) if len(sys.argv) > 3 else 0
    hi = int(sys.argv[3]) if len(sys.argv) > 3 else len(xs)
    lab = sys.argv[4] if len(sys.argv) > 4 else path
    rows = tails(xs, lo, hi, lab)
    fine(rows, lab)
