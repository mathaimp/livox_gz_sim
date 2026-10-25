# Copyright 2026 Mathai Mathew
# SPDX-License-Identifier: MIT

"""The mid360 and hap xacro macros expand to what the plugins expect."""
import math
import os
import xml.etree.ElementTree as ET

import pytest
import xacro

DEMO = os.path.join(os.path.dirname(__file__), '..', 'urdf', 'demo.urdf.xacro')


def expand(**args):
    doc = xacro.process_file(DEMO, mappings={k: str(v) for k, v in args.items()})
    return ET.fromstring(doc.toxml())


def sensor(robot, kind):
    found = robot.findall(f'.//sensor[@type="{kind}"]')
    assert len(found) == 1
    return found[0]


def joint_xyz(robot, name):
    xyz = robot.find(f'./joint[@name="{name}"]/origin').get('xyz')
    return [float(v) for v in xyz.split()]


def test_frames():
    robot = expand()
    assert joint_xyz(robot, 'livox_frame_joint') == pytest.approx([0, 0, 0.047])
    # IMU position in the point cloud frame, Mid-360 user manual
    assert joint_xyz(robot, 'livox_imu_joint') == pytest.approx([0.011, 0.02329, -0.04412])


def test_lidar_defaults():
    lidar = sensor(expand(), 'gpu_lidar')
    assert float(lidar.find('update_rate').text) == 10
    assert lidar.find('lidar/noise') is None  # the plugin adds the noise
    plugin = lidar.find('plugin')
    assert plugin.get('name') == 'livox_gz_sim::LivoxLidar'
    params = {e.tag: e.text for e in plugin}
    assert params['pattern'] == 'mid360.csv'
    assert params['samples'] == '20000'
    assert params['slices'] == '1'
    assert params['downsample'] == '1'
    assert params['topic'] == 'livox/lidar'
    assert params['frame_id'] == 'livox_frame'
    assert float(params['range_noise']) == 0.02
    assert float(params['angular_noise']) == 0.15


@pytest.mark.parametrize('slices, resolution, rate, h, v', [
    (1, 0.2, 10, 1801, 300),
    (5, 0.2, 50, 1801, 300),
    (10, 0.3, 100, 1201, 200),
    (10, 0.4, 100, 901, 150),
])
def test_slices_resolution(slices, resolution, rate, h, v):
    lidar = sensor(expand(slices=slices, resolution=resolution), 'gpu_lidar')
    assert float(lidar.find('update_rate').text) == rate
    assert lidar.find('plugin/slices').text == str(slices)
    scan = lidar.find('lidar/scan')
    for axis, n in (('horizontal', h), ('vertical', v)):
        assert int(scan.find(f'{axis}/samples').text) == n
        lo = float(scan.find(f'{axis}/min_angle').text)
        hi = float(scan.find(f'{axis}/max_angle').text)
        assert math.degrees(hi - lo) / (n - 1) == pytest.approx(resolution, rel=0.01)


def test_imu():
    imu = sensor(expand(), 'imu')
    assert float(imu.find('update_rate').text) == 200
    plugin = imu.find('plugin')
    assert plugin.get('name') == 'livox_gz_sim::LivoxImu'
    assert plugin.find('topic').text == 'livox/imu'
    assert plugin.find('frame_id').text == 'livox_frame'


def test_hap():
    robot = expand(model='hap')
    # point cloud origin: the window's front face, 23 mm above the bottom (STEP model, manual)
    assert joint_xyz(robot, 'livox_frame_joint') == pytest.approx([0.0523, 0.0012, 0.023])
    assert robot.findall('.//sensor[@type="imu"]') == []
    lidar = sensor(robot, 'gpu_lidar')
    params = {e.tag: e.text for e in lidar.find('plugin')}
    assert params['pattern'] == 'hap.csv'
    assert params['samples'] == '45200'  # 452k points/s at 10 Hz
    assert float(params['angular_noise']) == 0.1
    assert float(lidar.find('lidar/range/min').text) == 0.5
    assert float(lidar.find('lidar/range/max').text) == 150


@pytest.mark.parametrize('slices, resolution, rate, h, v', [
    (1, 0.2, 10, 605, 135),
    (10, 0.3, 100, 404, 90),
])
def test_hap_slices_resolution(slices, resolution, rate, h, v):
    lidar = sensor(expand(model='hap', slices=slices, resolution=resolution), 'gpu_lidar')
    assert float(lidar.find('update_rate').text) == rate
    scan = lidar.find('lidar/scan')
    assert int(scan.find('horizontal/samples').text) == h
    assert int(scan.find('vertical/samples').text) == v
