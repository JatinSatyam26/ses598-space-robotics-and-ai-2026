#!/usr/bin/env python3
import subprocess, time, re, csv, signal, sys, os

ENTITIES = ['drone', 'rover', 'habitat_dome']
OUTPUT   = os.environ.get('GT_LOG', '/tmp/phase7_groundtruth.csv')
RATE_HZ  = float(os.environ.get('GT_HZ', '2'))
RIG_X    = 5.0
RIG_Y    = -20.0

running = True
def _stop(sig, frame):
    global running
    running = False
signal.signal(signal.SIGTERM, _stop)
signal.signal(signal.SIGINT,  _stop)

def query_pose(entity):
    try:
        r = subprocess.run(
            ['gz', 'model', '-m', entity, '-p'],
            capture_output=True, text=True, timeout=2.0
        )
        lines = [l.strip() for l in r.stdout.splitlines() if l.strip().startswith('[')]
        if len(lines) >= 2:
            xyz = re.findall(r'(-?[0-9]+\.?[0-9]*(?:e[+-]?[0-9]+)?)', lines[0])
            rpy = re.findall(r'(-?[0-9]+\.?[0-9]*(?:e[+-]?[0-9]+)?)', lines[1])
            if len(xyz) == 3 and len(rpy) == 3:
                return [float(v) for v in xyz], [float(v) for v in rpy]
    except Exception:
        pass
    return None, None

os.makedirs(os.path.dirname(OUTPUT) if os.path.dirname(OUTPUT) else '.', exist_ok=True)
with open(OUTPUT, 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['wall_time', 'entity',
                'world_x', 'world_y', 'world_z',
                'rig_x', 'rig_y',
                'roll_rad', 'pitch_rad', 'yaw_rad'])
    f.flush()
    print(f'[gt_logger] writing to {OUTPUT} at {RATE_HZ} Hz', flush=True)

    interval = 1.0 / RATE_HZ
    while running:
        t0 = time.time()
        wt = f'{t0:.3f}'
        for entity in ENTITIES:
            xyz, rpy = query_pose(entity)
            if xyz and rpy:
                rx = round(xyz[0] - RIG_X, 6)
                ry = round(xyz[1] - RIG_Y, 6)
                w.writerow([wt, entity,
                             f'{xyz[0]:.6f}', f'{xyz[1]:.6f}', f'{xyz[2]:.6f}',
                             f'{rx:.6f}', f'{ry:.6f}',
                             f'{rpy[0]:.6f}', f'{rpy[1]:.6f}', f'{rpy[2]:.6f}'])
        f.flush()
        elapsed = time.time() - t0
        rem = interval - elapsed
        if rem > 0:
            time.sleep(rem)

print('[gt_logger] done.', flush=True)
