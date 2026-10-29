# -*- coding: ascii -*-
"""dump_window.py -- print raw telemetry frames in a time window, with a safe rate.

Usage:
    python dump_window.py <csv> <t0> <t1> [--tgt 787]

Columns printed:
  t[s]  err[deg]  rate[deg/s]  M

err  = shortest signed angle from upright, in degrees (0 = upright, +-180 = hanging)
rate = backward difference of the rod angle over the telemetry sample interval,
       computed on the UNWRAPPED angle so the +-180 deg seam does not produce
       a fake 3000 deg/s spike.  Rows where the seam is crossed are marked '~'.

That '~' matters: a naive difference on the wrapped angle reports spikes of
2000-3200 deg/s at every bottom crossing.  Those are artefacts, not motion.
"""

import io
import sys

CPD = 11.38


def main():
    if len(sys.argv) < 4:
        print(__doc__)
        return
    path = sys.argv[1]
    t0 = float(sys.argv[2])
    t1 = float(sys.argv[3])
    tgt = 787
    k = 4
    while k < len(sys.argv) - 1:
        if sys.argv[k] == "--tgt":
            tgt = int(sys.argv[k + 1])
        k += 1

    t12 = tgt * 4
    rows = []
    with io.open(path, "r", encoding="utf-8", errors="replace") as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln[0].isalpha():
                continue
            p = ln.split(",")
            if len(p) < 5:
                continue
            try:
                tt = float(p[0]); aa = int(p[1]); ss = int(p[2])
                ee = int(p[3]);   mm = int(p[4])
            except ValueError:
                continue
            if ss != 5:
                continue
            rows.append((tt, aa * 4, ee, mm))

    def err_of(a12):
        d = a12 - t12
        if d > 2048:
            d -= 4096
        elif d < -2048:
            d += 4096
        return d

    print("file : %s" % path)
    print("tgt  : %d (10-bit) = %d (12-bit) = %.1f deg" % (tgt, t12, t12 / CPD))
    print("")
    print("     t   err[deg]  rate[deg/s]   M")
    prev_t = prev_e = None
    for (tt, a12, _e, m) in rows:
        e = err_of(a12)
        if prev_e is None:
            rate, mark = 0.0, ""
        else:
            de = e - prev_e
            if de > 2048:
                de -= 4096
            elif de < -2048:
                de += 4096
            dt = tt - prev_t
            rate = (de / CPD / dt) if dt > 0 else 0.0
            # a real bottom crossing moves the wrapped angle by ~2048 counts
            mark = " ~" if abs(e - prev_e) > 1024 else ""
        if t0 <= tt <= t1:
            print(" %6.2f  %+8.1f   %+9.0f  %4d%s" % (tt, e / CPD, rate, m, mark))
        prev_t, prev_e = tt, e
    print("")
    print("'~' = the angle crossed the +-180 deg seam on this sample;")
    print("      the rate on that row is therefore only approximate.")


if __name__ == "__main__":
    main()
