# -*- coding: ascii -*-
"""phase_freq.py -- second-level analysis of one balance segment.

Companion to swing_score.py.  swing_score.py answers "how long did it hold";
this answers "what was it doing while it held".

Usage:
    python phase_freq.py <csv> [--tgt 787] [--zone 250] [--seg 0]

  --tgt   upright angle in 10-bit telemetry counts      (default 787)
  --zone  balance-zone half width in 12-bit counts      (default 250 = 22 deg)
  --seg   which balance segment to analyse, 0 = the longest   (default 0)

Prints
  * segment bounds, duration, rod peak-to-peak, mean|M|, saturation %
  * zero-crossing frequency and DFT peak of the rod angle
  * lag scan  corr(M, theta(t-lag))  for lag -80..+80 ms
      lag   0 ms  => M in phase with ANGLE  (stiffness / spring behaviour)
      lag ~1/4 T  => M in phase with RATE   (damping behaviour)
  * corr(M, E) at the same lags, to show whether the arm loop dominates

All angles are 12-bit counts: 4096 counts = 360 deg, 11.38 counts/deg.
Telemetry A is 10-bit, so it is multiplied by 4 on load.
"""

import io
import math
import sys

CPD = 11.38
ZONE = 250
MIN_SEG = 0.30


def load(path, tgt):
    t, a, e, m = [], [], [], []
    with io.open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line or line[0].isalpha():
                continue
            p = line.split(",")
            if len(p) < 5:
                continue
            try:
                tt = float(p[0]); aa = int(p[1]); ss = int(p[2])
                ee = int(p[3]);   mm = int(p[4])
            except ValueError:
                continue
            if ss != 5:
                continue
            t.append(tt); a.append(aa * 4); e.append(ee); m.append(mm)
    t12 = tgt * 4
    err = []
    for v in a:
        d = v - t12
        if d > 2048:
            d -= 4096
        elif d < -2048:
            d += 4096
        err.append(d)
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


def corr(x, y):
    n = min(len(x), len(y))
    if n < 8:
        return 0.0
    mx = sum(x[:n]) / n
    my = sum(y[:n]) / n
    sx = sum((v - mx) ** 2 for v in x[:n])
    sy = sum((v - my) ** 2 for v in y[:n])
    if sx <= 0 or sy <= 0:
        return 0.0
    return sum((x[k] - mx) * (y[k] - my) for k in range(n)) / math.sqrt(sx * sy)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return
    path = sys.argv[1]
    tgt, zone, want = 787, ZONE, 0
    k = 2
    while k < len(sys.argv) - 1:
        if sys.argv[k] == "--tgt":
            tgt = int(sys.argv[k + 1])
        elif sys.argv[k] == "--zone":
            zone = int(sys.argv[k + 1])
        elif sys.argv[k] == "--seg":
            want = int(sys.argv[k + 1])
        k += 1

    t, a, e, m, err = load(path, tgt)
    if not t:
        print("no T=5 frames in %s" % path)
        return
    segs = segments(t, err, zone, MIN_SEG)
    if not segs:
        print("no balance segment (|a_err| <= %d counts, >= %.2f s)" % (zone, MIN_SEG))
        return
    segs = sorted(segs, key=lambda ij: t[ij[1]] - t[ij[0]], reverse=True)
    if want >= len(segs):
        print("only %d segments, asked for #%d" % (len(segs), want))
        return
    i, j = segs[want]

    print("file      : %s" % path)
    print("segment   : #%d of %d (sorted by duration, longest first)" % (want, len(segs)))
    print("bounds    : t = %.2f .. %.2f s" % (t[i], t[j]))
    print("duration  : %.2f s" % (t[j] - t[i]))
    print("")

    seg = a[i:j + 1]
    print("rod p-p   : %.1f deg   (min %.1f, max %.1f)" % (
        (max(seg) - min(seg)) / CPD, min(seg) / CPD, max(seg) / CPD))
    mm = [abs(x) for x in m[i:j + 1]]
    print("mean|M|   : %.0f / 600  (%.0f%%)" % (
        sum(mm) / float(len(mm)), 100.0 * sum(mm) / float(len(mm)) / 600.0))
    print("saturated : %.1f%% of frames with |M| >= 590" % (
        100.0 * sum(1 for x in mm if x >= 590) / float(len(mm))))
    print("")

    th = [v / CPD for v in seg]
    mean_th = sum(th) / len(th)
    x = [v - mean_th for v in th]
    dur = t[j] - t[i]
    zc = sum(1 for q in range(1, len(x)) if x[q - 1] < 0.0 <= x[q])
    print("zero-crossings : %d over %.2f s  -> ~%.2f Hz" % (zc, dur, zc / dur if dur > 0 else 0))

    dt = 0.01
    bestf, bestp = 0.0, -1.0
    f10 = 2
    while f10 <= 450:
        fr = f10 * 0.1
        re = im = 0.0
        for q in range(len(x)):
            ang = 2.0 * math.pi * fr * q * dt
            re += x[q] * math.cos(ang)
            im += x[q] * math.sin(ang)
        pw = re * re + im * im
        if pw > bestp:
            bestp, bestf = pw, fr
        f10 += 1
    print("DFT peak       : %.1f Hz  (scan 0.2 .. 45 Hz)" % bestf)
    print("")

    print("lag scan (lag = how far theta is shifted back in time; M is not shifted)")
    print("  lag[ms]   corr(M,theta)   corr(M,E)")
    mm_l = [m[q] for q in range(i, j + 1)]
    ee_l = [e[q] for q in range(i, j + 1)]
    n = len(mm_l)
    lag = -8
    while lag <= 8:
        if lag >= 0:
            aa_ = mm_l[lag:]; bb = th[:n - lag]; cc = ee_l[:n - lag]
        else:
            aa_ = mm_l[:n + lag]; bb = th[-lag:]; cc = ee_l[-lag:]
        c1 = corr(aa_, bb)
        c2 = corr(aa_, cc)
        tag = ""
        if lag == 0:
            tag = "   <- in phase with ANGLE (stiffness)"
        if lag == 3 or lag == -3:
            tag = "   <- about 1/4 period (~angle RATE, damping)"
        print("  %6d   %12.3f   %9.3f%s" % (lag * 10, c1, c2, tag))
        lag += 1
    print("")
    print("reading: if corr(M,theta) at lag 0 is much larger than at lag ~+30 ms,")
    print("         the command is dominated by a stiffness term, not a damping term.")


if __name__ == "__main__":
    main()
