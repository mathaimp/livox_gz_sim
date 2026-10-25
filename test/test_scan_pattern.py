# Copyright 2026 Mathai Mathew
# SPDX-License-Identifier: MIT

"""The scan patterns load and fit the dense image their xacro renders."""
import math
import os
import xml.etree.ElementTree as ET

import pytest
import xacro

HERE = os.path.dirname(__file__)
DEMO = os.path.join(HERE, '..', 'urdf', 'demo.urdf.xacro')

# model: rows in the pattern file, rays per 10 Hz frame, FOV a frame must cover
# (azimuth, elevation, deg; None: all round)
MODELS = {
    'mid360': (800000, 20000, None, (-6.5, 51.5)),
    # Livox's recording was cut at 2^20 rows, 23.2 frames
    'hap': (1048575, 45200, (-59.5, 59.5), (-12.5, 12.5)),
}


def load(model):
    path = os.path.join(HERE, '..', 'scan_patterns', f'{model}.csv')
    with open(path) as f:
        assert f.readline().startswith('Time/s,Azimuth/deg,Zenith/deg')
        rays = [tuple(map(float, line.split(','))) for line in f if line.strip()]
    # azimuth in (-180, 180] as the plugin wraps it, elevation, deg
    return [(math.remainder(az, 360), 90 - zen) for _, az, zen in rays]


def dense_image(model):
    """(h_min, h_max, v_min, v_max) of the model's gpu_lidar, deg."""
    robot = ET.fromstring(xacro.process_file(DEMO, mappings={'model': model}).toxml())
    scan = robot.find('.//sensor[@type="gpu_lidar"]/lidar/scan')
    return [math.degrees(float(scan.find(f'{axis}/{end}_angle').text))
            for axis in ('horizontal', 'vertical') for end in ('min', 'max')]


@pytest.fixture(scope='module', params=list(MODELS))
def pattern(request):
    return request.param, load(request.param)


def test_size(pattern):
    model, rays = pattern
    assert len(rays) == MODELS[model][0]


def test_within_dense_image(pattern):
    # a ray outside the rendered image would be dropped
    model, rays = pattern
    h_min, h_max, v_min, v_max = dense_image(model)
    assert min(az for az, _ in rays) >= h_min
    assert max(az for az, _ in rays) <= h_max
    assert min(el for _, el in rays) >= v_min
    assert max(el for _, el in rays) <= v_max


def test_frame_covers_fov(pattern):
    model, rays = pattern
    _, frame, az_fov, el_fov = MODELS[model]
    for f in range(len(rays) // frame):
        chunk = rays[f * frame:(f + 1) * frame]
        elevations = [el for _, el in chunk]
        assert min(elevations) < el_fov[0] and max(elevations) > el_fov[1]
        if az_fov:
            azimuths = [az for az, _ in chunk]
            assert min(azimuths) < az_fov[0] and max(azimuths) > az_fov[1]
