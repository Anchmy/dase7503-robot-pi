#!/usr/bin/env python3
"""Generate a synthetic occupancy map of the DASE7503 game field.

The field is fixed and its dimensions are given in "QR Code and Gamefield.pdf"
(simplified drawing, all dimensions in mm).  Building the map from the drawing
means the team does not need a mapping run before Nav2 can be used, and the
rack coordinates in game_params.yaml share one origin with the map.

Layout (field frame, origin at the bottom-left corner of the field, x right,
y up, metres):

    field           3.02 x 2.00
    destination     x 0.00 .. 0.80, y 0.30 .. 1.70
    start           x 2.42 .. 3.02, y 0.60 .. 1.40
    racks           A (1.275, 1.580)  B (1.825, 0.510)
                    C (1.925, 1.720)  D (1.325, 0.200)

The rack centres come from the drawing's dimension chains:

    400   destination right edge -> A left edge
    650   A left edge -> C left edge
    450   destination right edge -> D left edge
    350   D right edge -> B left edge
    350   top wall -> A wall-side edge       (A is 140 mm deep)
    210   top wall -> C wall-side edge
    130   bottom wall -> D wall-side edge
    440   bottom wall -> B wall-side edge

Each rack footprint is taken as 150 x 140 mm, so the centre sits 70 mm from
the dimensioned edge.

Usage:
    python tools/make_field_map.py
    python tools/make_field_map.py --out-dir src/robot_navigation/maps
"""

import argparse
import os

FIELD_W = 3.02
FIELD_H = 2.00
RES = 0.05
ORIGIN = (-0.35, -0.35)
MAP_W = 75   # 3.75 m
MAP_H = 54   # 2.70 m

RACK_W = 0.15
RACK_D = 0.14
RACKS = {
    'A': (1.275, 1.580),
    'B': (1.825, 0.510),
    'C': (1.925, 1.720),
    'D': (1.325, 0.200),
}

FREE = 254
OCCUPIED = 0


def world_to_pixel(x, y):
    col = (x - ORIGIN[0]) / RES - 0.5
    row = MAP_H - (y - ORIGIN[1]) / RES - 0.5
    return col, row


def fill_rect(grid, x0, y0, x1, y1, value):
    for row in range(MAP_H):
        wy = ORIGIN[1] + (MAP_H - row - 0.5) * RES
        if not (y0 <= wy <= y1):
            continue
        for col in range(MAP_W):
            wx = ORIGIN[0] + (col + 0.5) * RES
            if x0 <= wx <= x1:
                grid[row][col] = value


def build_grid():
    grid = [[FREE] * MAP_W for _ in range(MAP_H)]

    # Walls, drawn just outside the field so the interior stays 3020x2000.
    t = 0.05
    fill_rect(grid, -t, -t, FIELD_W + t, 0.0, OCCUPIED)        # bottom
    fill_rect(grid, -t, FIELD_H, FIELD_W + t, FIELD_H + t, OCCUPIED)  # top
    fill_rect(grid, -t, -t, 0.0, FIELD_H + t, OCCUPIED)        # left
    fill_rect(grid, FIELD_W, -t, FIELD_W + t, FIELD_H + t, OCCUPIED)

    for x, y in RACKS.values():
        fill_rect(grid, x - RACK_W / 2, y - RACK_D / 2,
                  x + RACK_W / 2, y + RACK_D / 2, OCCUPIED)
    return grid


def write_pgm(grid, path):
    with open(path, 'wb') as f:
        f.write(f'P5\n{MAP_W} {MAP_H}\n255\n'.encode())
        for row in grid:
            f.write(bytes(row))


def write_yaml(path, image_name):
    text = (
        'image: {img}\n'
        'mode: trinary\n'
        'resolution: {res}\n'
        'origin: [{ox}, {oy}, 0.0]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.196\n'
    ).format(img=image_name, res=RES, ox=ORIGIN[0], oy=ORIGIN[1])
    with open(path, 'w', encoding='ascii') as f:
        f.write(text)


def ascii_preview(grid):
    for row in grid[::2]:
        print(''.join('#' if v == OCCUPIED else '.' for v in row[::1]))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--out-dir', default=os.path.join(
        os.path.dirname(__file__), '..', 'src', 'robot_navigation', 'maps'))
    parser.add_argument('--preview', action='store_true')
    args = parser.parse_args()

    out_dir = os.path.abspath(args.out_dir)
    os.makedirs(out_dir, exist_ok=True)
    grid = build_grid()

    pgm_path = os.path.join(out_dir, 'field.pgm')
    yaml_path = os.path.join(out_dir, 'field.yaml')
    write_pgm(grid, pgm_path)
    write_yaml(yaml_path, 'field.pgm')

    if args.preview:
        ascii_preview(grid)
    print(f'wrote {pgm_path} ({MAP_W}x{MAP_H} at {RES} m/px)')
    print(f'wrote {yaml_path}, origin {ORIGIN}')


if __name__ == '__main__':
    main()
