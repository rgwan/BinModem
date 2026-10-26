#!/usr/bin/env python3
"""Per-chord Ucode margin from a far-end capture (docs/244 step 1).

Every sample of the digital network's 8 kHz stream is put through the G.711
mu-law quantiser the central office's A/D applies, and the residual to the
nearest *codeword level* is reported in the half-step of that codeword's
chord.  Nothing here uses 349 LSB, and nothing fits a global sigma.
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
    """nearest-level mu-law encode of a 16-bit linear sample"""
    a = min(abs(int(x)), 32124)
    # bisect on magnitudes
    lo, hi = 0, len(MAGS) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if MAGS[mid] < a: lo = mid + 1
        else: hi = mid
    # lo is the first >= a
    cand = [lo]
    if lo > 0: cand.append(lo - 1)
    best = min(cand, key=lambda i: abs(MAGS[i] - a))
    lvl = LEVELS[best][0]
    return (-lvl if x < 0 else lvl), (LEVELS[best][1] & 0x7F) >> 4

# The chord's own step, measured from the alphabet: the smallest gap between
# two decode levels of the same chord.  No formula, no 349.
_TAB = {}
for _u in UCODES:
    _c = (_u & 0x7F) >> 4
    _TAB.setdefault(_c, []).append(abs(dec(_u)))
STEP = {}
for _c, _v in _TAB.items():
    _v = sorted(set(_v))
    STEP[_c] = min(b - a for a, b in zip(_v, _v[1:]))

def step_of(chord):
    return STEP[chord]

def load(path):
    a = array.array('h')
    with open(path, 'rb') as f:
        a.frombytes(f.read())
    if sys.byteorder == 'big':
        a.byteswap()
    return a

def report(xs, lo, hi, label):
    per = defaultdict(lambda: dict(n=0, s=0.0, mx=0.0, out=0, codes=set(), abssum=0.0))
    for k in range(lo, hi):
        x = xs[k]
        if x == 0:
            continue
        q, c = enc(x)
        e = x - q
        d = per[c]
        d['n'] += 1
        d['s'] += e * e
        d['abssum'] += abs(e)
        a = abs(e)
        if a > d['mx']: d['mx'] = a
        half = step_of(c) / 2.0
        if a > half: d['out'] += 1
        d['codes'].add(q)
    print(f"\n### {label}  samples {lo}..{hi} (={hi-lo})")
    print("chord step  n      rmsLSB rms/half  max/half  >half%  codes  minGap  gap/2/sig  marginLSB")
    tot = 0
    for c in sorted(per):
        d = per[c]
        st = step_of(c); half = st / 2.0
        rms = math.sqrt(d['s'] / d['n'])
        codes = sorted(d['codes'])
        gaps = [abs(b - a) for a, b in zip(codes, codes[1:])]
        g = min(gaps) if gaps else float('inf')
        marg = (g / 2.0) / rms if rms > 0 else float('inf')
        print(f"{c:5d} {st:5d} {d['n']:6d} {rms:7.2f} {rms/half:7.3f} {d['mx']/half:8.2f} "
              f"{100.0*d['out']/d['n']:6.2f} {len(codes):6d} {g:7.0f} {marg:10.2f} {marg*rms:10.1f}")
        tot += d['n']
    print(f"total {tot}")

if __name__ == '__main__':
    path = sys.argv[1]
    xs = load(path)
    print(f"{path}: {len(xs)} samples, {len(set(abs(x) for x in xs))} distinct |x|")
    if len(sys.argv) > 3:
        report(xs, int(sys.argv[2]), int(sys.argv[3]), "slice")
    else:
        report(xs, 0, len(xs), "whole capture")
