# Copyright 2026 Mathai Mathew
# SPDX-License-Identifier: MIT

"""The Mid-360 scan pattern loads and fits the dense image of urdf/mid360.xacro."""
import os

PATTERN = os.path.join(os.path.dirname(__file__), '..', 'scan_patterns', 'mid360.csv')
FRAME = 20000  # rays per 10 Hz frame (200k points/s)


def load():
    with open(PATTERN) as f:
        assert f.readline().startswith('Time/s,Azimuth/deg,Zenith/deg')
        rays = [tuple(map(float, line.split(','))) for line in f if line.strip()]
    return [(az, 90 - zen) for _, az, zen in rays]  # azimuth, elevation in deg


RAYS = load()


def test_size():
    # a whole number of frames, so every frame starts at the same phase of the pattern
    assert len(RAYS) == 800000
    assert len(RAYS) % FRAME == 0


def test_within_dense_image():
    # the gpu_lidar renders -7.4..52.4 deg; a ray outside it would be dropped
    elevations = [el for _, el in RAYS]
    assert min(elevations) >= -7.4
    assert max(elevations) <= 52.4
    assert all(0 <= az <= 360 for az, _ in RAYS)


def test_frame_covers_fov():
    # each frame sweeps the full Mid-360 FOV, -7..52 deg
    for f in range(len(RAYS) // FRAME):
        elevations = [el for _, el in RAYS[f * FRAME:(f + 1) * FRAME]]
        assert min(elevations) < -6.5
        assert max(elevations) > 51.5
