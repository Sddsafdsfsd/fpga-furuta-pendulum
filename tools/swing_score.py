# -*- coding: ascii -*-
"""swing_score.py -- score an automatic swing-up capture from a telemetry CSV.

Usage:
    python swing_score.py <csv> [--tgt 787] [--zone 250]

Reports, for the T=5 (enabled) part of the capture:
  * pump time      : E1 -> first real balance segment
  * balance segs   : runs where |a_err| stays inside +-zone counts for >=0.30 s
  * per segment    : duration, rod pp (deg), arm E start/end, arm drift (cnt/s),
                     mean |M|, saturation %
  * contact flags  : windows where the arm encoder is frozen while M != 0
                     (external contact / bind -- the run is suspect)

All angles are converted with the measured scale of 11.38 counts/degree (12-bit).
Exit code 0 always; read the printed verdict.
"""

import io
import sys

CPD = 11.38          # 12-bit counts per degree (4096 / 360)
ZONE = 250           # counts, ~22 deg
MIN_SEG = 0.30       # seconds


def load(path, tgt):
    t, a, e, m = [], [], [], []
    with io.open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line or line[0].isalpha():
                continue
            parts = line.split(",")
            if len(parts) < 5:
                continue
            try:
                tt = float(parts[0])
                aa = int(parts[1])
                ss = int(parts[2])
                ee = int(parts[3])
                mm = int(parts[4])
            except ValueError:
                continue
            if ss != 5:
                continue
            t.append(tt)
            a.append(aa * 4)          # telemetry A is 10-bit -> 12-bit
            e.append(ee)
            m.append(mm)
    n = len(t)
    t12 = tgt * 4                    # telemetry A is 10-bit, target given in 10-bit
    err = [0] * n
    for i in range(n):
        d = a[i] - t12
        if d > 2048:
            d -= 4096
        elif d < -2048:
            d += 4096
        err[i] = d
    return t, a, e, m, err


def segments(t, err, zone, min_len):
    out = []
    i, n = 0, len(t)
    while i < n:
        if abs(err[i]) <= zone:
            j = i
            while j + 1 < n and abs(err[j + 1]) <= zone:
                j += 1
            if t[j] - t[i] >= min_len:
                out.append((i, j))
            i = j + 1
        else:
            i += 1
    return out


def drift(t, e, i, j):
    if j <= i:
        return 0.0
    dt = t[j] - t[i]
    if dt <= 0:
        return 0.0
    return (e[j] - e[i]) / dt


def frozen_arm(t, e, m, i0, i1, max_dt=0.20, span=3):
    """Return list of (t_start, dur) where the arm's FULL range stays < span counts.

    Uses the running min/max of the window, not just the endpoints, so a slow
    wobble during pumping does not count as frozen.
    """
    n = min(i1, len(t))
    hits = []
    i = i0
    while i < n:
        j = i
        lo = hi = e[i]
        while j + 1 < n:
            v = e[j + 1]
            nlo = min(lo, v)
            nhi = max(hi, v)
            if nhi - nlo > span:
                break
            lo, hi = nlo, nhi
            j += 1
        if t[j] - t[i] >= max_dt:
            hits.append((t[i], t[j] - t[i]))
        i = j + 1
    return hits


def main():
    if len(sys.argv) < 2:
        print("usage: python swing_score.py <csv> [--tgt 787] [--zone 250]")
        return
    path = sys.argv[1]
    tgt = 787
    zone = ZONE
    for k in range(2, len(sys.argv) - 1):
        if sys.argv[k] == "--tgt":
            tgt = int(sys.argv[k + 1])
        elif sys.argv[k] == "--zone":
            zone = int(sys.argv[k + 1])

    t, a, e, m, err = load(path, tgt)
    if not t:
        print("no T=5 frames in %s" % path)
        return

    print("file      : %s" % path)
    print("T=5 span  : %.2f s  (%.0f frames)" % (t[-1] - t[0], len(t)))
    print("target    : %d counts (%.1f deg)" % (tgt, tgt / 4.0 * 360.0 / 1024.0))
    print("")

    segs = segments(t, err, zone, MIN_SEG)
    # a segment only counts as a balance if the motor was actually working:
    # M == 0 for the whole window means the arm was parked (rod resting on
    # something), not a controlled balance.
    real = []
    for (i, j) in segs:
        mm = [abs(x) for x in m[i:j + 1]]
        if mm and sum(mm) / float(len(mm)) >= 40:
            real.append((i, j))
        else:
            print("  [dropped] t=%.2f dur %.2f s : M==0 throughout (parked, not balancing)"
                  % (t[i], t[j] - t[i]))
    segs = real
    print("balance segments (|a_err| <= %d counts = %.0f deg, >= %.2f s): %d"
          % (zone, zone / CPD, MIN_SEG, len(segs)))
    if not segs:
        print("  none -- this run never held inside the zone")
        # still show the best approach
        best = min(range(len(err)), key=lambda i: abs(err[i]))
        print("  closest approach: %.1f deg at t=%.2f s" % (err[best] / CPD, t[best]))
    for (i, j) in segs:
        dur = t[j] - t[i]
        seg = a[i:j + 1]
        pp = (max(seg) - min(seg)) / float(CPD)
        mm = [abs(x) for x in m[i:j + 1]]
        sat = 100.0 * sum(1 for x in mm if x >= 590) / max(1, len(mm))
        mean_m = sum(mm) / float(max(1, len(mm)))
        print("  t=%7.2f  dur %5.2f s  rod pp %5.1f deg  E %+5d -> %+5d  drift %+7.1f cnt/s  mean|M| %3.0f (%2.0f%% sat)"
              % (t[i], dur, pp, e[i], e[j], drift(t, e, i, j), mean_m, sat))

    if segs:
        first = segs[0][0]
        print("")
        print("pump time : %.2f s  (E1 -> first balance segment)" % (t[first] - t[0]))

    worst = 0.0
    for (i, j) in segs:
        worst = max(worst, t[j] - t[i])
    print("best hold : %.2f s" % worst)

    hits = frozen_arm(t, e, m, 0, len(t))
    hits = [(tt, d) for (tt, d) in hits if d >= 0.20]
    if hits:
        print("")
        print("!! ARM FROZEN (|dE|<3 counts for >=0.20 s) -- check these windows:")
        print("   (a freeze WITH |M|>=100 means external contact/bind; with M~0 it is just idle)")
        for (tt, d) in hits[:6]:
            k = min(range(len(t)), key=lambda z: abs(t[z] - tt))
            mm = [abs(x) for x in m[k:k + int(d * 100) + 1]]
            tag = "M~0 idle" if (not mm or max(mm) < 100) else "M!=0  <== CONTACT?"
            print("   t=%7.2f  froze %.2f s   %s" % (tt, d, tag))
    else:
        print("contact   : no frozen-arm windows detected")

    print("")
    if worst >= 10.0:
        print("VERDICT   : solid capture (>=10 s). Reproduce it before changing code.")
    elif worst >= 3.0:
        print("VERDICT   : real capture but short (%.1f s). This is the reproducible baseline." % worst)
    elif worst > 0:
        print("VERDICT   : touching only (%.1f s). Not yet a capture." % worst)
    else:
        print("VERDICT   : no capture in this run.")


if __name__ == "__main__":
    main()
