#!/usr/bin/env python
# -*- coding: ascii -*-
"""
balance_score.py -- score one balance attempt from its CSV.   (v2, reviewer patch)

WHY
    "how many seconds did it stand" is the least informative number in the log.
    Four signatures say whether the loop is healthy:

      1. t_release - t_E1   the release should land 150 ms BEFORE E1 .. 50 ms
                            after it.  Later than that and the arm has already
                            run away before the pendulum is free.
      2. E at E1 frame      |E| < 60.  Larger = the arm already ran away.
      3. E net drift        < 100 counts over the BALANCE segment.  A monotonic
                            drift means the arm position loop is dead (stiction
                            swallows kp_e*|E|) and no gain change fixes it.
      4. sign(M) at in-zone |E|>200 must agree with sign(A - tgt).  This is the
                            ONLY signature that says the v2.6/v2.7 soft-limit
                            fix works; before it, M was pinned at -400.

    v2 CHANGES (all four were calibrated on v2.4's failure mode and lie on v2.7):
      * balance window 30 -> 50 counts.  At 30 the KNOWN-GOOD 11.8 s capture
        scores 8.4 s, because the normal limit cycle is already +-26 counts.
        A ">10 s" acceptance test built on win=30 can never be passed.
      * |E|>200 check is now restricted to IN-ZONE frames (|A-tgt|<=140).
        On v2.4 all 11 such frames were out of zone (rod at 90 deg); on v2.7
        those same frames are driven by park_cmd (-sign(E)*350), not by the
        pendulum law, so the old check reports 0/11 even when the fix works.
      * E net drift is measured over the balance segment only.  park_cmd drags
        the arm home after a failure, which the old metric scored as "drift".
      * M zero-crossing filter ignores M==0 frames (gate closed / park deadband
        on v2.7), which would otherwise drag the setpoint estimate off.

USAGE
    python balance_score.py rel_1.csv
    python balance_score.py rel_*.csv --tgt 368     # v2.7: always pass this
                                                    # (compiled setpoint 1472/4)
"""

import io
import sys
import glob

IN_ZONE = 140          # 10-bit counts == BAL_ZONE(560) in 12-bit; ~49 deg
WIN = 50               # 10-bit counts, ~17.6 deg; normal limit cycle is +-26


def read_csv(path):
    rows = []
    with io.open(path, encoding='utf-8', errors='replace') as f:
        for line in f:
            p = [x for x in line.replace(',', ' ').split() if x]
            if len(p) < 5:
                continue
            try:
                rows.append((float(p[0]), int(p[1]), int(p[2]), int(p[3]), int(p[4])))
            except ValueError:
                continue
    return rows


def score(path, tgt=None):
    rows = read_csv(path)
    if len(rows) < 200:
        print('%s : too few frames' % path.split('\\')[-1])
        return None
    t = [r[0] for r in rows]
    A = [r[1] for r in rows]
    T = [r[2] for r in rows]
    E = [r[3] for r in rows]
    M = [r[4] for r in rows]
    n = len(rows)
    dt = (t[-1] - t[0]) / max(n - 1, 1)

    name = path.split('\\')[-1]
    print('=== %s   (%d frames, %.1f s, %.0f Hz) ===' % (name, n, t[-1] - t[0], 1.0 / dt if dt else 0))

    # ---- E1 ---------------------------------------------------------------
    i_e1 = next((i for i in range(n) if T[i] == 5), None)
    if i_e1 is None:
        print('  no T==5 frame: the loop was never enabled, nothing to score.')
        return None
    t_e1 = t[i_e1]
    print('  t_E1            : %.2f s' % t_e1)

    # ---- release: first sustained motion of A near E1 ---------------------
    thr = 15                     # sum of |dA| over 5 frames (~5 deg in 50 ms)
    i_rel = None
    lo = max(6, i_e1 - int(1.2 / dt))
    for i in range(lo, min(n, i_e1 + int(1.2 / dt))):
        s = sum(abs(A[k] - A[k - 1]) for k in range(i - 5, i))
        if s >= thr:
            i_rel = i
            break
    if i_rel is None:
        print('  t_release       : NOT FOUND (A never moved near E1)')
    else:
        d = (t[i_rel] - t_e1) * 1000.0
        if d > 150 or d < -150:
            tag = 'BAD: %s' % ('pull the stop EARLIER' if d > 0 else 'too early, rod fell far')
        elif d > 50:
            tag = 'MARGINAL (late, but usable)'
        else:
            tag = 'OK -150..+50'
        print('  t_release       : %.2f s   delta = %+.0f ms   [%s]' % (t[i_rel], d, tag))

    # ---- E at release -----------------------------------------------------
    e_rel = E[i_e1]
    print('  E at E1 frame   : %+d counts   [%s]' % (
        e_rel, 'OK |E|<60' if abs(e_rel) < 60 else 'BAD: arm already ran away'))

    # ---- setpoint estimate (M zero crossings, M==0 frames excluded) -------
    bal = [r for r in rows if r[2] == 5]
    zs = [bal[i][1] for i in range(1, len(bal))
          if bal[i - 1][4] != 0 and bal[i][4] != 0
          and abs(bal[i - 1][4]) >= 100 and abs(bal[i][4]) >= 100
          and (bal[i - 1][4] < 0) != (bal[i][4] < 0)]
    est = (sum(zs) / len(zs)) if zs else None
    if tgt is None:
        tgt = int(round(est)) if est else None
    print('  tgt estimated   : %s (10-bit) from %d M zero-crossings%s' % (
        ('%d' % tgt) if tgt is not None else 'n/a', len(zs),
        '' if len(zs) >= 20 else '  [WEAK: <20 crossings, pass --tgt explicitly]'))

    # ---- balance duration + the segment it occupies -----------------------
    # ---- v5: report EVERY balance segment, not just the longest ----------
    #   v33_a proved the old code could report a false "survived 13.8 s":
    #   the arm-limit event was at t=16.26 inside a segment that ended at
    #   16.43, while the *longest* segment (a hand re-placement 8 s later)
    #   ran to t=40.  Comparing the two gave a win that never happened.
    segs = []
    if tgt is not None:
        cur = 0
        start = 0
        for k, r in enumerate(bal):
            if abs(r[1] - tgt) <= WIN:
                if cur == 0:
                    start = k
                cur += 1
            else:
                if cur * dt >= 1.0:
                    segs.append((start, k - 1))
                cur = 0
        if cur * dt >= 1.0:
            segs.append((start, len(bal) - 1))
        segs.sort(key=lambda s: s[1] - s[0], reverse=True)
        print('  balance segments:')
        for s0, s1 in segs[:4]:
            dur = (bal[s1][0] - bal[s0][0])
            e0, e1 = bal[s0][3], bal[s1][3]
            print('     t=%5.2f..%5.2f  %5.1f s   E %+5d -> %+5d  (%+.0f cnt/s)'
                  % (bal[s0][0], bal[s1][0], dur, e0, e1, (e1 - e0) / max(dur, 1e-9)))
        if segs:
            s0, s1 = segs[0]
            seg = (s0, s1)
            print('  balance time    : %.1f s  (longest, anywhere in the capture)'
                  % (bal[s1][0] - bal[s0][0]))

    # ---- E net drift over the balance segment only ------------------------
    #   park_cmd (v2.7) deliberately drags the arm home after a failure; scoring
    #   the whole T==5 window would call that "drift" and blame the position loop.
    #   v6: judge the RATE, not the net.  A 17 s run can travel 211 counts
    #   (=-12 cnt/s, healthy) and a 8 s run can travel 542 (=+68 cnt/s, dead);
    #   the old "<100 counts" rule flagged the good one as BAD.
    if seg[1] > seg[0] and (seg[1] - seg[0]) * dt > 0.5:
        drift = bal[seg[1]][3] - bal[seg[0]][3]
        dur = bal[seg[1]][0] - bal[seg[0]][0]
        rate = drift / max(dur, 1e-9)
        ok = abs(rate) < 40.0
        print('  E drift rate    : %+.1f counts/s   (%+d counts over %.1f s)   [%s]' % (
            rate, drift, dur,
            'OK: |rate|<40' if ok else 'BAD: the arm walks out on its own'))

        # ---- THE signature: sign(M) vs the P + arm-position terms --------
        #   v3: the old test compared sign(M) with sign(A-tgt) only.  That is
        #   WRONG whenever the rod is moving, because the D term legitimately
        #   dominates (kd_a*a_rate) and can override the P term.  On rel_1.csv
        #   it produced a "17/22 FAIL" that was a false alarm.
        #   Fix: only score frames where the rod is nearly still, and compare
        #   against the exact duty the P + position + soft-limit-bias terms
        #   would command (kp_a=472, kp_e=228, SOFT_REV=200, all Q8).
        bad = tot = 0
        first_bad = None
        for k in range(max(seg[0], 1), seg[1] + 1):
            r = bal[k]
            if abs(r[1] - tgt) > IN_ZONE:      # out of zone -> park owns it
                continue
            if abs(r[3]) <= 200 or r[4] == 0:
                continue
            if abs(r[1] - bal[k - 1][1]) > 1:  # rod moving -> D term rules
                continue
            if abs(r[3] - bal[k - 1][3]) > 1:  # arm moving -> kd_e term rules
                continue
            exp = (472.0 / 256.0) * (r[1] - tgt) * 4.0 + (228.0 / 256.0) * r[3]
            exp -= 200.0 if r[3] > 0 else -200.0        # soft-limit bias
            tot += 1
            if (exp > 0) != (r[4] > 0):
                bad += 1
                if first_bad is None:
                    first_bad = r
        if tot == 0:
            print('  |E|>200 check   : no slow IN-ZONE frame at |E|>200')
            print('                    -> vacuous pass; use the bench test instead')
        else:
            print('  |E|>200 check   : M sign correct %d/%d   [%s]' % (
                tot - bad, tot,
                'OK' if bad == 0 else 'FAIL: soft-limit still fights the pendulum'))
            if first_bad:
                print('     first bad frame: t=%.2f A=%d E=%d M=%d' % (
                    first_bad[0], first_bad[1], first_bad[3], first_bad[4]))
    else:
        print('  balance segment : <0.5 s -- arm/drift signatures NOT scored')
        print('                    (a run that never stood cannot say anything',
              'about the arm loop)')

    # ---- where did the run die?  (v30+: the arm limit must NOT steal control)
    ARM_HARD = 450
    hit = next((r for r in bal if abs(r[3]) > ARM_HARD), None)
    if hit is not None:
        print('  arm limit       : first |E|>%d at t=%.2f (%.2f s after E1) A=%d E=%+d M=%+d'
              % (ARM_HARD, hit[0], hit[0] - t_e1, hit[1], hit[3], hit[4]))
    else:
        print('  arm limit       : never reached |E|>%d (best travel %d)'
              % (ARM_HARD, max([abs(r[3]) for r in bal]) if bal else 0))

    # ---- v5: did the rod survive THE arm-limit event that hit THIS run? ---
    if segs and hit is not None:
        owner = None
        for s0, s1 in segs:
            if bal[s0][0] - 0.5 <= hit[0] <= bal[s1][0] + 0.5:
                owner = (s0, s1)
                break
        if owner is None:
            print('  verdict         : the arm limit was hit while the rod was already out')
        else:
            gap = bal[owner[1]][0] - hit[0]
            if gap < 1.0:
                print('  verdict         : DIED at the arm limit (%.1f s after the hit)'
                      % gap)
            elif gap < 3.0:
                print('  verdict         : MARGINAL -- %.1f s after the hit' % gap)
            else:
                print('  verdict         : SURVIVED the arm limit for %.1f s' % gap)
    elif hit is None:
        print('  verdict         : never hit the arm limit')

    # ---- mean |M| and saturation -----------------------------------------
    if bal:
        mm = [abs(r[4]) for r in bal]
        print('  mean |M|        : %.0f / 600 = %.0f%%   saturated %.0f%%' % (
            sum(mm) / len(mm), 100.0 * sum(mm) / len(mm) / 600.0,
            100.0 * sum(1 for x in mm if x >= 399) / len(mm)))

    print('')


def main():
    if len(sys.argv) < 2:
        sys.exit('usage: balance_score.py <csv> [--tgt N]')
    tgt = None
    args = sys.argv[1:]
    if '--tgt' in args:
        k = args.index('--tgt')
        tgt = int(args[k + 1])
        del args[k:k + 2]          # v2: do not treat the value as a filename
    files = []
    for a in args:
        if a.startswith('--'):
            continue
        g = glob.glob(a)
        files.extend(g if g else [a])
    files = [f for f in files if not f.endswith('.py')]
    for f in files:
        try:
            score(f, tgt)
        except FileNotFoundError:
            print('%s : not found' % f)


if __name__ == '__main__':
    main()
