"""F710“X模式验ID、D模式控制、拔插复位”单元测试。"""

import pytest

from mobile_joy_control import controller_binding as binding


def _make_usb_device(root,
                     name,
                     product,
                     serial='',
                     devpath='2.1.3',
                     devnum='19'):
    device = root / name
    device.mkdir()
    (device / 'idVendor').write_text('046d\n', encoding='utf-8')
    (device / 'idProduct').write_text(product + '\n', encoding='utf-8')
    (device / 'devpath').write_text(devpath + '\n', encoding='utf-8')
    (device / 'devnum').write_text(devnum + '\n', encoding='utf-8')
    if serial:
        (device / 'serial').write_text(serial + '\n', encoding='utf-8')
    return device


def _switch_mode(receiver, product, serial=''):
    """模拟同一 USB 端口上的 X/D 模式重新枚举。"""
    (receiver / 'idProduct').write_text(product + '\n', encoding='utf-8')
    serial_file = receiver / 'serial'
    if serial:
        serial_file.write_text(serial + '\n', encoding='utf-8')
    elif serial_file.exists():
        serial_file.unlink()


def _remove_usb_device(receiver):
    for child in receiver.iterdir():
        child.unlink()
    receiver.rmdir()


def _make_joy_device(root, input_root, receiver, name='js0'):
    joy_dev = root / name
    joy_dev.write_text('', encoding='utf-8')
    input_entry = input_root / name
    input_entry.mkdir(parents=True)
    (input_entry / 'device').symlink_to(receiver)
    return joy_dev


@pytest.fixture
def controller_files(tmp_path):
    usb_root = tmp_path / 'usb'
    usb_root.mkdir()
    hardware = tmp_path / 'hardware.yaml'
    hardware.write_text(
        'components:\n  pc1:\n    controller:\n'
        '      unique_id: "046d:71879d8f"\n',
        encoding='utf-8')
    return usb_root, hardware


def test_matching_xinput_waits_for_dinput(controller_files):
    usb_root, hardware = controller_files
    _make_usb_device(usb_root, '1-2.1.3', 'c21f', '71879D8F')
    validator = binding.ControllerInsertionValidator(hardware, usb_root)

    assert validator.poll() is True
    assert validator.binding_ok is False
    assert validator.stage == binding.ControllerValidationStage.WAITING_FOR_DINPUT
    assert validator.bound_id == '046d:71879D8F'
    assert 'X 模式 ID 校验通过' in validator.reason


def test_matching_xinput_then_dinput_authorizes_control(controller_files):
    usb_root, hardware = controller_files
    receiver = _make_usb_device(usb_root, '1-2.1.3', 'c21f', '71879D8F')
    validator = binding.ControllerInsertionValidator(hardware, usb_root)

    assert validator.poll() is True
    _switch_mode(receiver, 'c219')
    assert validator.poll() is True
    assert validator.binding_ok is True
    assert validator.stage == binding.ControllerValidationStage.AUTHORIZED
    assert validator.current.input_mode == 'DInput'
    assert validator.current.serial_source == 'validated_xinput_session'


def test_xinput_empty_then_same_port_dinput_authorizes_within_switch_window(
        controller_files):
    usb_root, hardware = controller_files
    receiver = _make_usb_device(
        usb_root, '1-2.1.3', 'c21f', '71879D8F')
    clock = [0.0]
    validator = binding.ControllerInsertionValidator(
        hardware,
        usb_root,
        monotonic=lambda: clock[0],
        mode_switch_timeout_s=2.0,
    )

    assert validator.poll() is True
    _remove_usb_device(receiver)
    assert validator.poll() is True
    assert validator.stage == \
        binding.ControllerValidationStage.SWITCHING_TO_DINPUT
    assert validator.binding_ok is False
    assert validator.bound_id == '046d:71879D8F'
    assert 'X→D 模式切换中' in validator.reason

    clock[0] = 0.782
    _make_usb_device(
        usb_root, '1-2.1.3', 'c219', devpath='2.1.3', devnum='20')
    assert validator.poll() is True
    assert validator.stage == binding.ControllerValidationStage.AUTHORIZED
    assert validator.binding_ok is True


def test_xinput_to_dinput_switch_timeout_clears_validation(controller_files):
    usb_root, hardware = controller_files
    receiver = _make_usb_device(
        usb_root, '1-2.1.3', 'c21f', '71879D8F')
    clock = [0.0]
    validator = binding.ControllerInsertionValidator(
        hardware,
        usb_root,
        monotonic=lambda: clock[0],
        mode_switch_timeout_s=2.0,
    )

    assert validator.poll() is True
    _remove_usb_device(receiver)
    assert validator.poll() is True
    clock[0] = 1.999
    assert validator.poll() is False
    assert validator.stage == \
        binding.ControllerValidationStage.SWITCHING_TO_DINPUT

    clock[0] = 2.0
    assert validator.poll() is True
    assert validator.stage == binding.ControllerValidationStage.WAITING_FOR_XINPUT
    assert validator.binding_ok is False
    assert validator.bound_id == ''
    assert '切换超过 2.0秒' in validator.reason

    _make_usb_device(
        usb_root, '1-2.1.3', 'c219', devpath='2.1.3', devnum='20')
    assert validator.poll() is True
    assert validator.stage == binding.ControllerValidationStage.WAITING_FOR_XINPUT
    assert validator.binding_ok is False


def test_incomplete_sysfs_snapshot_starts_switch_window(controller_files):
    usb_root, hardware = controller_files
    receiver = _make_usb_device(
        usb_root, '1-2.1.3', 'c21f', '71879D8F')
    clock = [0.0]
    validator = binding.ControllerInsertionValidator(
        hardware,
        usb_root,
        monotonic=lambda: clock[0],
        mode_switch_timeout_s=2.0,
    )

    assert validator.poll() is True
    (receiver / 'devpath').unlink()
    assert validator.poll() is True
    assert validator.stage == \
        binding.ControllerValidationStage.SWITCHING_TO_DINPUT
    assert validator.binding_ok is False

    clock[0] = 0.8
    (receiver / 'devpath').write_text('2.1.3\n', encoding='utf-8')
    _switch_mode(receiver, 'c219')
    assert validator.poll() is True
    assert validator.stage == binding.ControllerValidationStage.AUTHORIZED
    assert validator.binding_ok is True


def test_dinput_usb_within_window_can_finish_joy_enumeration_later(
        controller_files, tmp_path):
    usb_root, hardware = controller_files
    receiver = _make_usb_device(
        usb_root, '1-2.1.3', 'c21f', '71879D8F')
    input_root = tmp_path / 'input_sysfs'
    joy_root = tmp_path / 'dev_input'
    input_root.mkdir()
    joy_root.mkdir()
    joy_dev = joy_root / 'js0'
    clock = [0.0]
    validator = binding.ControllerInsertionValidator(
        hardware,
        usb_root,
        joy_dev,
        input_root,
        monotonic=lambda: clock[0],
        mode_switch_timeout_s=2.0,
    )

    assert validator.poll() is True
    _remove_usb_device(receiver)
    assert validator.poll() is True

    clock[0] = 0.8
    receiver = _make_usb_device(
        usb_root, '1-2.1.3', 'c219', devpath='2.1.3', devnum='20')
    assert validator.poll() is False
    assert validator.stage == \
        binding.ControllerValidationStage.DINPUT_ENUMERATING
    assert validator.binding_ok is False

    # USB已在2秒内出现，joy输入设备即使稍后才就绪也不算模式切换超时。
    clock[0] = 2.5
    assert validator.poll() is False
    assert validator.stage == \
        binding.ControllerValidationStage.DINPUT_ENUMERATING

    _make_joy_device(joy_root, input_root, receiver)
    assert validator.poll() is True
    assert validator.stage == binding.ControllerValidationStage.AUTHORIZED
    assert validator.binding_ok is True


@pytest.mark.parametrize('timeout_s', [0.0, -1.0, float('inf'), float('nan')])
def test_mode_switch_timeout_must_be_positive_and_finite(
        controller_files, timeout_s):
    usb_root, hardware = controller_files
    with pytest.raises(ValueError, match='mode_switch_timeout_s'):
        binding.ControllerInsertionValidator(
            hardware,
            usb_root,
            mode_switch_timeout_s=timeout_s,
        )


def test_dinput_without_current_xinput_validation_is_rejected(controller_files):
    usb_root, hardware = controller_files
    _make_usb_device(usb_root, '1-2.1.3', 'c219')
    validator = binding.ControllerInsertionValidator(hardware, usb_root)

    assert validator.poll() is True
    assert validator.binding_ok is False
    assert validator.stage == binding.ControllerValidationStage.WAITING_FOR_XINPUT
    assert '尚未完成 X 模式 ID 校验' in validator.reason
    assert validator.poll() is False


@pytest.mark.parametrize(
    'controller_yaml',
    [
        '    controller:\n      model: "F710"\n',
        '    controller:\n      unique_id: ""\n',
        '    controller:\n      unique_id: "046d:c219:unbound"\n',
        '    controller:\n      unique_id: " 046D:C219:UNBOUND "\n',
    ],
)
def test_missing_or_unbound_id_requires_application_pairing(
        tmp_path, controller_yaml):
    usb_root = tmp_path / 'usb'
    usb_root.mkdir()
    receiver = _make_usb_device(
        usb_root, '1-2.1.3', 'c21f', '71879D8F')
    hardware = tmp_path / 'hardware.yaml'
    hardware.write_text(
        'components:\n  pc1:\n' + controller_yaml,
        encoding='utf-8')
    validator = binding.ControllerInsertionValidator(hardware, usb_root)

    assert validator.poll() is True
    assert validator.binding_ok is False
    assert validator.pairing_required is True
    assert validator.stage == binding.ControllerValidationStage.PAIRING_REQUIRED
    assert validator.reason == (
        '手柄尚未采集绑定 ID：'
        'components.pc1.controller.unique_id 缺失或为空；'
        '请保持手柄处于 X 模式并通过 application 采集 ID，'
        '采集完成后将手柄切到 D 模式，再切回 X 模式触发校验')

    # application 写入后，同一个X实例不会静默放行，必须按提示重新切换模式。
    hardware.write_text(
        'components:\n  pc1:\n    controller:\n'
        '      unique_id: "046d:71879D8F"\n',
        encoding='utf-8')
    assert validator.poll() is False

    _switch_mode(receiver, 'c219')
    assert validator.poll() is True
    assert validator.stage == binding.ControllerValidationStage.WAITING_FOR_XINPUT
    assert validator.binding_ok is False

    _switch_mode(receiver, 'c21f', '71879D8F')
    assert validator.poll() is True
    assert validator.stage == binding.ControllerValidationStage.WAITING_FOR_DINPUT
    assert validator.bound_id == '046d:71879D8F'

    _switch_mode(receiver, 'c219')
    assert validator.poll() is True
    assert validator.binding_ok is True
    assert validator.stage == binding.ControllerValidationStage.AUTHORIZED


def test_other_invalid_bound_id_remains_rejected(tmp_path):
    usb_root = tmp_path / 'usb'
    usb_root.mkdir()
    _make_usb_device(usb_root, '1-2.1.3', 'c21f', '71879D8F')
    hardware = tmp_path / 'hardware.yaml'
    hardware.write_text(
        'components:\n  pc1:\n    controller:\n      unique_id: 123\n',
        encoding='utf-8')
    validator = binding.ControllerInsertionValidator(hardware, usb_root)

    assert validator.poll() is True
    assert validator.binding_ok is False
    assert validator.pairing_required is False
    assert validator.stage == binding.ControllerValidationStage.REJECTED
    assert 'unique_id 为空或格式错误' in validator.reason


def test_mismatched_xinput_id_never_authorizes_dinput(controller_files):
    usb_root, hardware = controller_files
    receiver = _make_usb_device(usb_root, '1-2.1.3', 'c21f', 'AAAAAAAA')
    validator = binding.ControllerInsertionValidator(hardware, usb_root)

    assert validator.poll() is True
    assert validator.stage == binding.ControllerValidationStage.REJECTED
    assert '配对失败' in validator.reason

    _switch_mode(receiver, 'c219')
    assert validator.poll() is True
    assert validator.binding_ok is False
    assert validator.stage == binding.ControllerValidationStage.WAITING_FOR_XINPUT


def test_receiver_removal_revokes_authorization(controller_files):
    usb_root, hardware = controller_files
    receiver = _make_usb_device(usb_root, '1-2.1.3', 'c21f', '71879D8F')
    validator = binding.ControllerInsertionValidator(hardware, usb_root)
    assert validator.poll() is True
    _switch_mode(receiver, 'c219')
    assert validator.poll() is True
    assert validator.binding_ok is True

    _remove_usb_device(receiver)
    assert validator.poll() is True
    assert validator.binding_ok is False
    assert validator.stage == binding.ControllerValidationStage.WAITING_FOR_XINPUT
    assert '重新插入后必须先用 X 模式校验 ID' in validator.reason


def test_reinserted_dinput_cannot_reuse_previous_authorization(
        controller_files):
    usb_root, hardware = controller_files
    receiver = _make_usb_device(usb_root, '1-2.1.3', 'c21f', '71879D8F')
    validator = binding.ControllerInsertionValidator(hardware, usb_root)
    assert validator.poll() is True
    _switch_mode(receiver, 'c219')
    assert validator.poll() is True
    assert validator.binding_ok is True

    _remove_usb_device(receiver)
    assert validator.poll() is True
    _make_usb_device(usb_root, '1-2.1.3', 'c219', devnum='20')
    assert validator.poll() is True
    assert validator.binding_ok is False
    assert validator.stage == binding.ControllerValidationStage.WAITING_FOR_XINPUT


def test_replaced_dinput_instance_revokes_authorization(controller_files):
    usb_root, hardware = controller_files
    receiver = _make_usb_device(usb_root, '1-2.1.3', 'c21f', '71879D8F')
    validator = binding.ControllerInsertionValidator(hardware, usb_root)
    assert validator.poll() is True
    _switch_mode(receiver, 'c219')
    assert validator.poll() is True
    assert validator.binding_ok is True

    # 不经过空扫描，直接在同一路径替换实例，USB设备号变化也必须撤销授权。
    _remove_usb_device(receiver)
    _make_usb_device(usb_root, '1-2.1.3', 'c219', devnum='20')
    assert validator.poll() is True
    assert validator.binding_ok is False
    assert '实例发生变化' in validator.reason


def test_dinput_must_use_same_usb_port_as_validated_xinput(controller_files):
    usb_root, hardware = controller_files
    receiver = _make_usb_device(usb_root, '1-2.1.3', 'c21f', '71879D8F')
    validator = binding.ControllerInsertionValidator(hardware, usb_root)
    assert validator.poll() is True

    _remove_usb_device(receiver)
    _make_usb_device(usb_root, '1-2.1.4', 'c219', devpath='2.1.4')
    assert validator.poll() is True
    assert validator.binding_ok is False
    assert validator.stage == binding.ControllerValidationStage.REJECTED
    assert '不在刚才通过 X 模式校验的 USB 端口' in validator.reason


def test_xinput_without_serial_waits_and_recovers(controller_files):
    usb_root, hardware = controller_files
    receiver = _make_usb_device(usb_root, '1-2.1.3', 'c21f')
    clock = [0.0]
    validator = binding.ControllerInsertionValidator(
        hardware,
        usb_root,
        enumeration_timeout_s=1.0,
        monotonic=lambda: clock[0],
    )

    assert validator.poll() is False
    assert validator.binding_ok is False
    clock[0] = 1.0
    assert validator.poll() is True
    assert '序列号尚未就绪' in validator.reason

    (receiver / 'serial').write_text('71879D8F\n', encoding='utf-8')
    assert validator.poll() is True
    assert validator.stage == binding.ControllerValidationStage.WAITING_FOR_DINPUT


def test_dinput_waits_for_joy_device_and_recovers(controller_files, tmp_path):
    usb_root, hardware = controller_files
    receiver = _make_usb_device(usb_root, '1-2.1.3', 'c21f', '71879D8F')
    input_root = tmp_path / 'input_sysfs'
    joy_root = tmp_path / 'dev_input'
    input_root.mkdir()
    joy_root.mkdir()
    joy_dev = joy_root / 'js0'
    clock = [0.0]
    validator = binding.ControllerInsertionValidator(
        hardware,
        usb_root,
        joy_dev,
        input_root,
        enumeration_timeout_s=1.0,
        monotonic=lambda: clock[0],
    )

    assert validator.poll() is True
    _switch_mode(receiver, 'c219')
    assert validator.poll() is False
    clock[0] = 1.0
    assert validator.poll() is True
    assert validator.binding_ok is False

    _make_joy_device(joy_root, input_root, receiver)
    assert validator.poll() is True
    assert validator.binding_ok is True


def test_unrelated_joy_device_is_not_authorized(controller_files, tmp_path):
    usb_root, hardware = controller_files
    receiver = _make_usb_device(usb_root, '1-2.1.3', 'c21f', '71879D8F')
    unrelated_receiver = tmp_path / 'unrelated_receiver'
    unrelated_receiver.mkdir()
    input_root = tmp_path / 'input_sysfs'
    joy_root = tmp_path / 'dev_input'
    input_root.mkdir()
    joy_root.mkdir()
    joy_dev = _make_joy_device(joy_root, input_root, unrelated_receiver)
    clock = [0.0]
    validator = binding.ControllerInsertionValidator(
        hardware,
        usb_root,
        joy_dev,
        input_root,
        enumeration_timeout_s=1.0,
        monotonic=lambda: clock[0],
    )

    assert validator.poll() is True
    _switch_mode(receiver, 'c219')
    assert validator.poll() is False
    clock[0] = 1.0
    assert validator.poll() is True
    assert validator.binding_ok is False
    assert '不属于本次已校验的接收器端口' in validator.reason


def test_multiple_receivers_are_rejected(controller_files):
    usb_root, hardware = controller_files
    _make_usb_device(usb_root, '1-2.1.3', 'c21f', '71879D8F')
    _make_usb_device(usb_root, '1-2.1.4', 'c21f', 'AAAAAAAA')
    validator = binding.ControllerInsertionValidator(hardware, usb_root)

    assert validator.poll() is True
    assert validator.binding_ok is False
    assert validator.stage == binding.ControllerValidationStage.REJECTED
    assert '多个 F710 接收器' in validator.reason
