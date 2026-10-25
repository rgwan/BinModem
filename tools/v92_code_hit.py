"""Code-hit rate per capture, and whether the far end's levels are on the grid.

Input: `V92_B1U_LEVELS` files from the replay of docs/105 §6:

    BINMODEM_V92_PCM_UP=1 MODEM_RAW=/home/hikari/v92up-runs/<tag>.agr-tx.raw \
    MODEM_FS=8000 V92_PRE=none V92_UP_RATE=24000 V92_B1U_GATE=off V92_FE=1 \
    V92_B1U_LEVELS=/tmp/cg/old/<tag>.levels V92_ETA_TRACE=/tmp/cg/old/<tag>.eta \
    cargo test -q -p modem --release --test replay -- --ignored --nocapture probe_replay_raw

One `now level mode` line per symbol; the level is the equaliser's output in the
units `set_data_mode` installed, i.e. `G x a(eta)`, so `level / G` is the
design's own `a(eta)` scale and the ladder step is two a-units.

The LSB scale is not fitted. `[8.8.3]` makes the far end transmit at its desired
power, `docs/25` §2 measures that at `rms = 1609 LSB`, and `G` was picked to make
`ms(v G) = 1` against the *set's* mean square -- so one a-unit is
`1609 * G = 174.5 LSB` at LC = 8 by construction, which is `docs/103` §2's
conversion (`0.101 half-grid = 17.6 LSB`).

Two measurements per capture:

  * `res` / `half-grid` -- how far the received level is from the level the
    design asked for, in LSB and in the design's own half-gaps;
  * `code-hit` -- the fraction of symbols within a quarter of the *local codec
    cell* of that level. A request on a code centre has half a cell either side,
    so a quarter cell is that margin halved again for the noise.

And the one that decides whether the residual is a *snapping* residual at all:
the per-level mean of `level/G - (2 eta + 1)`. Snapping to codes puts a
systematic offset on each level -- at 174.5 LSB/a-unit those offsets are
`{5.5, 0.5, 3.5, 22.5, 6.5, 35.5, 32.5, 2.5}` LSB, i.e. +0.13 and +0.20 a-units
on eta 5 and 6. Noise puts no mean on any level.
"""
import glob, sys
import numpy as np

def dec(m):
    c, s = m >> 4, m & 15
    return 4 * (((2 * s + 33) << c) - 33)

CODES = np.array([dec(m) for m in range(128)], dtype=float)
GAIN = 0.1084652289        # upchoice::gain_for(8): the odd lattice at LC = 8
UNIT = 1609.0 * GAIN       # LSB per a-unit, by 8.8.3 + docs/25 §2
WINDOW = 4800              # 400 data frames, 20x docs/72 §6's window

def cell_of(v):
    m = np.clip(np.searchsorted(CODES, v), 1, 127)
    below = CODES[m] - CODES[m - 1]
    above = np.where(m + 1 < 128, CODES[np.minimum(m + 1, 127)] - CODES[m], below)
    return np.maximum(8.0, 0.5 * (below + above))

def read(path, window=WINDOW):
    value, mode = [], []
    for line in open(path):
        f = line.split()
        if len(f) != 3:
            continue
        value.append(float(f[1])); mode.append(f[2])
    value = np.array(value); mode = np.array(mode)
    data = np.abs(value[mode == 'data'])
    if len(data) < 576:
        return np.abs(value[mode == 'score'])
    return data[:window]

def analyse(path, lc=8):
    mags = read(path)
    a = mags / GAIN
    eta = np.clip(np.round((a - 1.0) / 2.0), 0, lc - 1)
    err_a = a - (2.0 * eta + 1.0)
    asked = (2.0 * eta + 1.0) * UNIT
    err = np.abs(err_a) * UNIT
    half = float(np.sqrt(np.mean(err_a ** 2)))
    hit = float(np.mean(err <= 0.25 * cell_of(asked)))
    means = [float(err_a[eta == e].mean()) if np.any(eta == e) else float('nan') for e in range(lc)]
    return dict(tag=path.split('/')[-1].replace('.levels', ''), n=len(mags),
                res=float(np.sqrt(np.mean(err ** 2))), half=half,
                cell=float(np.mean(err / cell_of(asked))), hit=hit, means=means)

def predicted_offsets(lc=8):
    """What snapping to codes would put on each level, in a-units."""
    points = np.array([2 * e + 1 for e in range(lc)], dtype=float) * UNIT
    values = np.unique(CODES[1:])
    near = np.array([values[np.argmin(np.abs(values - p))] for p in points])
    return (near - points) / UNIT

if __name__ == "__main__":
    rows = [analyse(p) for p in sorted(glob.glob(sys.argv[1] if len(sys.argv) > 1 else '/tmp/cg/old/*.levels'))]
    print(f"{'capture':<9}{'syms':>6}{'res LSB':>8}{'half-grid':>10}{'res/cell':>9}{'code-hit':>10}{'code grid':>11}")
    for r in rows:
        print(f"{r['tag']:<9}{r['n']:>6}{r['res']:>8.1f}{r['half']:>10.3f}{r['cell']:>9.2f}"
              f"{100*r['hit']:>9.1f}%{'100.0%':>11}")
    print(f"\nmedian: res {np.median([r['res'] for r in rows]):.1f} LSB, "
          f"half-grid {np.median([r['half'] for r in rows]):.3f}, "
          f"code-hit {100*np.median([r['hit'] for r in rows]):.1f}%")
    print("\nper-level mean of (level/G - a(eta)), in a-units -- what a snapping")
    print("residual would put there, against what the captures show:")
    snap = predicted_offsets()
    print(f"{'capture':<9}" + "".join(f"{'eta'+str(e):>8}" for e in range(8)))
    for r in rows[:3]:
        print(f"{r['tag']:<9}" + "".join(f"{m:>+8.3f}" for m in r['means']))
    print(f"{'snapping':<9}" + "".join(f"{m:>+8.3f}" for m in snap))
    print(f"{'+-2se':<9}" + "".join(f"{2*0.11/np.sqrt(max(1,r['n']/8)):>8.3f}" for _ in range(8)))
