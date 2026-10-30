import importlib.util
from pathlib import Path

import pytest
import yaml
from launch_ros.actions import Node


@pytest.fixture
def cb_launch():
    launch_path = (Path(__file__).resolve().parents[1] / 'launch' /
                   'command_board.launch.py')
    spec = importlib.util.spec_from_file_location('command_board_launch',
                                                  launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('backend, expected_parameters, expected_executables', [
    ('dexe', ['P', {
        'chassis_backend': 'dexe'
    }], ['command_board_node', 'map_manager_node', 'map_asset_manager_node']),
    ('cloudia', ['P', 'O', {
        'chassis_backend': 'cloudia'
    }], ['command_board_node']),
])
def test_backend_nodes(cb_launch, monkeypatch, backend, expected_parameters,
                       expected_executables):
    assert cb_launch.command_board_parameters(backend, 'P',
                                              'O') == (expected_parameters)
    calls = []
    original = cb_launch.command_board_parameters

    def record_parameters(*args):
        calls.append(args)
        return original(*args)

    monkeypatch.setattr(cb_launch, 'command_board_parameters',
                        record_parameters)
    nodes = cb_launch.build_nodes(backend, 'P', 'M', 'O')
    assert calls == [(backend, 'P', 'O')]
    assert len(nodes) == len(expected_executables)
    assert all(isinstance(node, Node) for node in nodes)
    assert [node.node_executable for node in nodes] == expected_executables


@pytest.mark.parametrize('backend', ['', 'nav2', 'Cloudia'])
def test_invalid_backend(cb_launch, backend):
    with pytest.raises(ValueError,
                       match='chassis_backend must be dexe or cloudia'):
        cb_launch.command_board_parameters(backend, 'P', 'O')
    with pytest.raises(ValueError,
                       match='chassis_backend must be dexe or cloudia'):
        cb_launch.build_nodes(backend, 'P', 'M', 'O')


def test_cloudia_overrides():
    overrides_path = (Path(__file__).resolve().parents[1] / 'params' /
                      'command_board_cloudia_overrides.yaml')
    content = overrides_path.read_text(encoding='utf-8')
    assert content.splitlines()[0].startswith('#')
    assert yaml.safe_load(content) == {
        '/**': {
            'ros__parameters': {
                'sensor_monitor': {
                    'enable': False
                },
                'nav2': {
                    'autostart': False
                },
                'path_navigation': {
                    'autostart': False
                },
            }
        }
    }
