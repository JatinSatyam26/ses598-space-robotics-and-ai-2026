#!/usr/bin/env python3
"""
Phase 12 analysis — computes odom vs TRN localization error against
ground truth from /tmp/phase12_poses.csv.

Usage:
    python3 scripts/phase12_analysis.py [/tmp/phase12_poses.csv]

Output:
    - Summary stats table (printed)
    - Per-10s sample table (printed)
    - /tmp/phase12_error.png  (if matplotlib available)
"""

import sys
import csv
import math
import os

INPUT = sys.argv[1] if len(sys.argv) > 1 else '/tmp/phase12_poses.csv'


def read_csv(path):
    gt, odom, trn = [], [], []
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            t = float(row['wall_time'])
            x = float(row['x'])
            y = float(row['y'])
            src = row['source']
            if src == 'gt':
                gt.append((t, x, y))
            elif src == 'odom':
                odom.append((t, x, y))
            elif src == 'trn':
                trn.append((t, x, y))
    return gt, odom, trn


def nearest(samples, t):
    """Return sample from list nearest in time to t."""
    if not samples:
        return None
    return min(samples, key=lambda s: abs(s[0] - t))


def dist(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def compute_bias(gt, estimates, t_start, window=10.0):
    """
    Compute mean (est - gt) offset during first `window` seconds.
    This accounts for the known ~1.5m Y-offset between DiffDrive odom
    (starts at 0,0) and rig-relative GT (rover spawns at rig 0,1.5).
    """
    t_end = t_start + window
    bx, by, n = 0.0, 0.0, 0
    for tg, gx, gy in gt:
        if tg > t_end:
            break
        sample = nearest(estimates, tg)
        if sample and abs(sample[0] - tg) < 1.0:
            bx += sample[1] - gx
            by += sample[2] - gy
            n += 1
    if n == 0:
        return 0.0, 0.0
    return bx / n, by / n


def main():
    if not os.path.exists(INPUT):
        print(f'ERROR: {INPUT} not found. Run the demo with Phase 12 logger first.')
        sys.exit(1)

    gt, odom, trn = read_csv(INPUT)

    if not gt:
        print('ERROR: no ground truth rows in CSV.')
        sys.exit(1)

    print(f'Loaded: {len(gt)} GT samples, {len(odom)} odom samples, {len(trn)} TRN samples')
    print(f'Mission duration: {gt[-1][0] - gt[0][0]:.1f}s')
    print()

    t0 = gt[0][0]

    # Compute initial bias (odom starts at 0,0; GT starts at ~0,1.5)
    odom_bx, odom_by = compute_bias(gt, odom, t0, window=8.0)
    trn_bx, trn_by = compute_bias(gt, trn, t0, window=8.0)
    # TRN doesn't publish until the map arrives (~60s into mission), so its
    # bias cannot be estimated from the first 8s.  Fall back to odom bias,
    # since both sources start from the same DiffDrive odom origin.
    if trn_bx == 0.0 and trn_by == 0.0 and (odom_bx != 0.0 or odom_by != 0.0):
        trn_bx, trn_by = odom_bx, odom_by
        trn_bias_src = 'inherited from odom (TRN not available in first 8s)'
    else:
        trn_bias_src = 'estimated from data'
    print(f'Initial bias (est - GT, first 8s):')
    print(f'  odom: ({odom_bx:+.3f}, {odom_by:+.3f}) m')
    print(f'  TRN:  ({trn_bx:+.3f}, {trn_by:+.3f}) m  [{trn_bias_src}]')
    print()

    # Compute per-GT-sample errors (bias-corrected)
    odom_errors, trn_errors, times_rel = [], [], []
    print(f'{"Time(s)":>8}  {"GT_x":>7} {"GT_y":>7}  '
          f'{"Odom_err(m)":>11}  {"TRN_err(m)":>10}  {"Improve%":>8}')
    print('-' * 68)

    sample_every = max(1, len(gt) // 20)  # print ~20 rows
    for i, (tg, gx, gy) in enumerate(gt):
        t_rel = tg - t0

        om = nearest(odom, tg)
        tr = nearest(trn, tg)

        oe = dist(om[1] - odom_bx, om[2] - odom_by, gx, gy) if om and abs(om[0] - tg) < 1.0 else None
        te = dist(tr[1] - trn_bx, tr[2] - trn_by, gx, gy) if tr and abs(tr[0] - tg) < 1.0 else None

        if oe is not None:
            odom_errors.append(oe)
        if te is not None:
            trn_errors.append(te)
        times_rel.append(t_rel)

        if i % sample_every == 0 or i == len(gt) - 1:
            oe_s = f'{oe:.3f}' if oe is not None else '  ---'
            te_s = f'{te:.3f}' if te is not None else '  ---'
            if oe is not None and te is not None and oe > 1e-6:
                imp = f'{(oe - te) / oe * 100:+.1f}%'
            else:
                imp = '    ---'
            print(f'{t_rel:8.1f}  {gx:7.3f} {gy:7.3f}  {oe_s:>11}  {te_s:>10}  {imp:>8}')

    print()

    # Summary stats
    if odom_errors and trn_errors:
        oe_mean = sum(odom_errors) / len(odom_errors)
        te_mean = sum(trn_errors) / len(trn_errors)
        oe_max = max(odom_errors)
        te_max = max(trn_errors)
        oe_fin = odom_errors[-1]
        te_fin = trn_errors[-1]
        improvement = (oe_mean - te_mean) / oe_mean * 100 if oe_mean > 1e-6 else 0.0

        print('=' * 50)
        print(f'{"Metric":<25} {"Odom":>10} {"TRN":>10}')
        print('-' * 50)
        print(f'{"Mean error (m)":<25} {oe_mean:>10.3f} {te_mean:>10.3f}')
        print(f'{"Max error (m)":<25} {oe_max:>10.3f} {te_max:>10.3f}')
        print(f'{"Final error (m)":<25} {oe_fin:>10.3f} {te_fin:>10.3f}')
        print(f'{"Mean improvement":<25} {"---":>10} {improvement:>+9.1f}%')
        print('=' * 50)

    # Optional matplotlib plot
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(figsize=(10, 5))

        # Downsample to GT timestamps
        t_axis = [g[0] - t0 for g in gt]
        oe_plot = []
        te_plot = []
        for tg, gx, gy in gt:
            om = nearest(odom, tg)
            tr = nearest(trn, tg)
            oe_plot.append(
                dist(om[1] - odom_bx, om[2] - odom_by, gx, gy)
                if om and abs(om[0] - tg) < 1.0 else float('nan'))
            te_plot.append(
                dist(tr[1] - trn_bx, tr[2] - trn_by, gx, gy)
                if tr and abs(tr[0] - tg) < 1.0 else float('nan'))

        ax.plot(t_axis, oe_plot, label='Odom error', color='tab:orange', linewidth=1.5)
        ax.plot(t_axis, te_plot, label='TRN error', color='tab:blue', linewidth=1.5)
        ax.set_xlabel('Mission time (s)')
        ax.set_ylabel('Localization error (m)')
        ax.set_title('Phase 12: Rover Localization Error — Odom vs TRN-Corrected')
        ax.legend()
        ax.grid(True, alpha=0.4)
        fig.tight_layout()
        out_png = '/tmp/phase12_error.png'
        fig.savefig(out_png, dpi=120)
        print(f'\nPlot saved: {out_png}')
    except ImportError:
        print('\nmatplotlib not available — skipping plot')
    except Exception as e:
        print(f'\nPlot failed: {e}')


if __name__ == '__main__':
    main()
