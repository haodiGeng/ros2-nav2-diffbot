"""Summarize sampled chassis / caster clearance against the fixed test box.

This is a planar, discrete geometric check, not contact-sensor evidence.
The checked geometry is a 0.9 x 0.64 m chassis and four 0.06 m caster
spheres centered at (+/-0.45, +/-0.27). Other attachments are not checked.
"""
import json
import math
from pathlib import Path
import sys


def chassis_overlap(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    u, v = (c, s), (-s, c)
    for ax, ay in (u, v, (1., 0.), (0., 1.)):
        center_gap = abs((x - 1.2) * ax + y * ay)
        robot_extent = .45 * abs(u[0] * ax + u[1] * ay)
        robot_extent += .32 * abs(v[0] * ax + v[1] * ay)
        box_extent = .2 * (abs(ax) + abs(ay))
        if center_gap > robot_extent + box_extent:
            return False
    return True


def caster_overlap(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    for wx in (-.45, .45):
        for wy in (-.27, .27):
            cx, cy = x + c * wx - s * wy, y + s * wx + c * wy
            dx, dy = max(abs(cx - 1.2) - .2, 0.), max(abs(cy) - .2, 0.)
            if math.hypot(dx, dy) <= .06:
                return True
    return False


for filename in sys.argv[1:]:
    path = Path(filename)
    output = []
    for trial in json.loads(path.read_text()):
        samples = trial['odometry']
        summary = {k: trial.get(k) for k in (
            'goal', 'status', 'wall_seconds', 'recoveries',
            'max_localization_position_error')}
        summary['samples'] = len(samples)
        summary['sampled_chassis_overlap'] = sum(
            chassis_overlap(*sample[:3]) for sample in samples)
        summary['sampled_caster_overlap'] = sum(
            caster_overlap(*sample[:3]) for sample in samples)
        summary['path_length'] = sum(math.hypot(b[0]-a[0], b[1]-a[1])
                                     for a, b in zip(samples, samples[1:]))
        output.append(summary)
    target = path.with_name(path.stem + '_summary.json')
    target.write_text(json.dumps(output, indent=2))
    print(path.name, json.dumps(output), flush=True)
