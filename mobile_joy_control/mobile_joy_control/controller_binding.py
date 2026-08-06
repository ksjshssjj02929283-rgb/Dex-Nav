"""F710 接收器的 XInput 身份校验与 DInput 控制授权。"""

import math
import re
import time
from dataclasses import dataclass
from enum import Enum, auto
from pathlib import Path

import yaml

SUPPORTED_F710_VENDOR = '046d'
F710_PRODUCT_XINPUT = 'c21f'
F710_PRODUCT_DINPUT = 'c219'
SUPPORTED_F710_PRODUCTS = {F710_PRODUCT_XINPUT, F710_PRODUCT_DINPUT}
F710_INPUT_MODES = {
    F710_PRODUCT_XINPUT: 'XInput',
    F710_PRODUCT_DINPUT: 'DInput',
}

DEFAULT_HARDWARE_CONFIG_FILE = '/etc/dexe_robot/hardware.yaml'
# 保留旧参数，避免已有 launch 命令因参数消失而失败；新流程不再用缓存授权。
DEFAULT_CONTROLLER_SERIAL_CACHE_FILE = (Path.home() / '.cache' / 'encription' /
                                        'controller_serials.json')
USB_SYSFS = Path('/sys/bus/usb/devices')
INPUT_SYSFS = Path('/sys/class/input')
DEFAULT_RECEIVER_ENUMERATION_TIMEOUT_S = 2.0  # D模式输入设备枚举等待时间，单位：秒。
DEFAULT_MODE_SWITCH_TIMEOUT_S = 2.0  # X切D后接收器重新枚举的最长时间，单位：秒。


class ControllerNotFoundError(RuntimeError):
    """USB 枚举中没有发现 F710 接收器。"""


class ControllerEnumerationPendingError(RuntimeError):
    """接收器已出现，但输入设备枚举尚未完成。"""


class ControllerBindingMismatchError(RuntimeError):
    """X模式读取的当前接收器 ID 与本地绑定 ID 不一致。"""


class ControllerBindingRequiredError(RuntimeError):
    """hardware.yaml 尚未保存手柄绑定 ID，需要 application 采集。"""


class ControllerValidationStage(Enum):
    """一次接收器插入周期内的授权状态。"""

    WAITING_FOR_XINPUT = auto()
    PAIRING_REQUIRED = auto()
    WAITING_FOR_DINPUT = auto()
    SWITCHING_TO_DINPUT = auto()
    DINPUT_ENUMERATING = auto()
    AUTHORIZED = auto()
    REJECTED = auto()


def validation_stage_to_string(stage):
    """返回稳定的状态名称，供日志和测试使用。"""
    return stage.name


@dataclass(frozen=True)
class ControllerIdentity:
    """用于身份校验和日志输出的 F710 身份快照。"""

    unique_id: str
    product: str
    input_mode: str
    serial_source: str
    identity_verified: bool
    usb_path: str
    usb_sysfs_name: str


@dataclass(frozen=True)
class ControllerReceiverInstance:
    """用于检测模式变化和物理拔插的 USB 实例快照。"""

    usb_sysfs_name: str
    inode: int
    device_number: str
    product: str
    usb_path: str
    serial: str


def _read_text(path):
    """读取 sysfs 单行字段；字段不存在或为空时返回空字符串。"""
    try:
        return path.read_text(encoding='utf-8').strip()
    except OSError:
        return ''


def _normalize_binding_id(value):
    """统一 F710 绑定 ID 格式，避免大小写差异造成误判。"""
    if not isinstance(value, str):
        return ''

    match = re.fullmatch(r'([0-9a-fA-F]{4}):([^:\s]+)', value.strip())
    if match is None:
        return ''
    return f'{match.group(1).lower()}:{match.group(2).upper()}'


def load_bound_controller_id(hardware_config_file):
    """读取 hardware.yaml 中 pc1.controller.unique_id。"""
    config_path = Path(hardware_config_file)
    if not config_path.is_file():
        raise RuntimeError(f'硬件配置不存在: {config_path}')

    try:
        config = yaml.safe_load(config_path.read_text(encoding='utf-8'))
    except (OSError, yaml.YAMLError) as exc:
        raise RuntimeError(f'硬件配置读取失败: {exc}') from exc

    try:
        controller = config['components']['pc1']['controller']
    except (KeyError, TypeError) as exc:
        raise RuntimeError('硬件配置缺少 components.pc1.controller') from exc
    if not isinstance(controller, dict):
        raise RuntimeError('components.pc1.controller 格式错误')

    unique_id = controller.get('unique_id')
    unique_id_text = unique_id.strip() if isinstance(unique_id, str) else ''
    unbound_id = (f'{SUPPORTED_F710_VENDOR}:{F710_PRODUCT_DINPUT}:unbound')
    if (unique_id is None or
            (isinstance(unique_id, str) and
             (not unique_id_text or unique_id_text.lower() == unbound_id))):
        raise ControllerBindingRequiredError(
            '手柄尚未采集绑定 ID：'
            'components.pc1.controller.unique_id 缺失或为空；'
            '请保持手柄处于 X 模式并通过 application 采集 ID，'
            '采集完成后将手柄切到 D 模式，再切回 X 模式触发校验')

    normalized_id = _normalize_binding_id(unique_id)
    if not normalized_id:
        raise RuntimeError('components.pc1.controller.unique_id 为空或格式错误')
    if not normalized_id.startswith(f'{SUPPORTED_F710_VENDOR}:'):
        raise RuntimeError(f'绑定设备不是当前支持的手柄: {normalized_id}')
    return normalized_id


def scan_f710_receiver_instances(usb_sysfs=USB_SYSFS):
    """读取当前所有 F710 接收器及其真实 USB 模式。"""
    usb_sysfs = Path(usb_sysfs)
    if not usb_sysfs.is_dir():
        raise RuntimeError(f'USB 设备目录不存在: {usb_sysfs}')

    try:
        usb_entries = list(usb_sysfs.iterdir())
    except OSError as exc:
        raise RuntimeError(f'USB 设备目录读取失败: {exc}') from exc

    instances = []
    for entry in usb_entries:
        vendor = _read_text(entry / 'idVendor').lower()
        product = _read_text(entry / 'idProduct').lower()
        if (vendor != SUPPORTED_F710_VENDOR or
                product not in SUPPORTED_F710_PRODUCTS):
            continue
        try:
            inode = entry.stat().st_ino
        except OSError:
            # 扫描时刚好拔出，下个周期再按稳定结果处理。
            continue
        device_number = _read_text(entry / 'devnum')
        usb_path = _read_text(entry / 'devpath')
        if not device_number or not usb_path:
            # USB移除过程中sysfs字段会逐个消失，不完整快照按暂时离线处理。
            continue

        instances.append(
            ControllerReceiverInstance(
                usb_sysfs_name=entry.name,
                inode=inode,
                device_number=device_number,
                product=product,
                usb_path=usb_path,
                serial=_read_text(entry / 'serial'),
            ))

    return tuple(sorted(instances, key=lambda item: item.usb_sysfs_name))


def read_current_controller_identity(usb_sysfs=USB_SYSFS):
    """在 X 模式读取唯一 F710 的真实序列号。"""
    instances = scan_f710_receiver_instances(usb_sysfs)
    if not instances:
        raise ControllerNotFoundError('未检测到当前支持的手柄 USB 接收器')
    if len(instances) > 1:
        device_list = ', '.join(
            f'{F710_INPUT_MODES[item.product]}@{item.usb_path}'
            for item in instances)
        raise RuntimeError(f'检测到多个受支持的手柄，无法唯一选择: {device_list}')

    receiver = instances[0]
    if receiver.product != F710_PRODUCT_XINPUT:
        raise RuntimeError('当前手柄处于 D 模式，无法读取真实 ID；请先切换到 X 模式')
    if not receiver.serial:
        raise ControllerEnumerationPendingError('X 模式接收器已出现，但真实序列号尚未就绪')

    unique_id = _normalize_binding_id(
        f'{SUPPORTED_F710_VENDOR}:{receiver.serial}')
    if not unique_id:
        raise RuntimeError(f'X 模式读取到的手柄序列号格式错误: {receiver.serial}')
    return ControllerIdentity(
        unique_id=unique_id,
        product=receiver.product,
        input_mode=F710_INPUT_MODES[receiver.product],
        serial_source='usb',
        identity_verified=True,
        usb_path=receiver.usb_path,
        usb_sysfs_name=receiver.usb_sysfs_name,
    )


def _validate_joy_device(receiver, joy_dev, usb_sysfs, input_sysfs):
    """确认 joy_node 打开的 D 模式输入设备属于已校验的 USB 端口。"""
    if not joy_dev:
        return

    joy_path = Path(joy_dev).expanduser()
    try:
        resolved_joy_path = joy_path.resolve(strict=True)
    except OSError as exc:
        raise ControllerEnumerationPendingError(
            f'D 模式手柄输入设备尚未就绪: {joy_path}') from exc

    input_device_path = Path(input_sysfs) / resolved_joy_path.name / 'device'
    try:
        resolved_input_device = input_device_path.resolve(strict=True)
        receiver_path = (Path(usb_sysfs) /
                         receiver.usb_sysfs_name).resolve(strict=True)
    except OSError as exc:
        raise ControllerEnumerationPendingError(
            f'D 模式手柄输入设备 sysfs 尚未就绪: {input_device_path}') \
            from exc

    if (resolved_input_device != receiver_path and
            receiver_path not in resolved_input_device.parents):
        raise ControllerEnumerationPendingError(
            f'手柄输入设备 {joy_path} 不属于本次已校验的接收器端口 '
            f'{receiver.usb_sysfs_name}')


def check_controller_binding(hardware_config_file,
                             usb_sysfs=USB_SYSFS,
                             **_unused_kwargs):
    """仅在 X 模式读取真实 ID 并与 hardware.yaml 比较。"""
    bound_id = load_bound_controller_id(hardware_config_file)
    current = read_current_controller_identity(usb_sysfs)
    if current.unique_id != bound_id:
        raise ControllerBindingMismatchError(
            f'手柄配对失败，绑定 ID={bound_id}，当前 ID={current.unique_id}')
    return bound_id, current


class ControllerInsertionValidator:
    """执行“X模式验ID、切D模式放行、拔出后复位”的安全状态机。"""

    def __init__(self,
                 hardware_config_file,
                 usb_sysfs=USB_SYSFS,
                 joy_dev=None,
                 input_sysfs=INPUT_SYSFS,
                 enumeration_timeout_s=DEFAULT_RECEIVER_ENUMERATION_TIMEOUT_S,
                 monotonic=time.monotonic,
                 serial_cache_file=None,
                 mode_switch_timeout_s=DEFAULT_MODE_SWITCH_TIMEOUT_S):
        self.hardware_config_file = hardware_config_file
        # 兼容旧调用参数；D模式缓存不再参与本次插入的授权判断。
        self.serial_cache_file = serial_cache_file
        self.usb_sysfs = Path(usb_sysfs)
        self.joy_dev = joy_dev
        self.input_sysfs = Path(input_sysfs)
        self.enumeration_timeout_s = enumeration_timeout_s
        try:
            mode_switch_timeout_s = float(mode_switch_timeout_s)
        except (TypeError, ValueError) as exc:
            raise ValueError('mode_switch_timeout_s 必须是数值') from exc
        if (not math.isfinite(mode_switch_timeout_s) or
                mode_switch_timeout_s <= 0.0):
            raise ValueError('mode_switch_timeout_s 必须是大于0的有限数值')
        self.mode_switch_timeout_s = mode_switch_timeout_s
        self.monotonic = monotonic
        self._receiver_instances = ()
        self._validated_usb_path = ''
        self._validated_usb_sysfs_name = ''
        self._validation_deadline = 0.0
        self._mode_switch_deadline = 0.0
        self._enumeration_timeout_reported = False
        self.stage = ControllerValidationStage.WAITING_FOR_XINPUT
        self.binding_ok = False
        self.reason = '请插入 F710 接收器，并先将手柄切换到 X 模式校验 ID'
        self.bound_id = ''
        self.current = None

    def _reset_for_xinput(self, reason):
        """撤销旧授权；物理拔插后必须重新进行 X 模式校验。"""
        self.stage = ControllerValidationStage.WAITING_FOR_XINPUT
        self.binding_ok = False
        self.reason = reason
        self.bound_id = ''
        self.current = None
        self._validated_usb_path = ''
        self._validated_usb_sysfs_name = ''
        self._mode_switch_deadline = 0.0
        self._enumeration_timeout_reported = False

    def _reject(self, reason):
        """锁定本次错误状态，直到接收器拔出或重新进入 X 模式。"""
        self.stage = ControllerValidationStage.REJECTED
        self.binding_ok = False
        self.reason = reason
        self.current = None
        self._validated_usb_path = ''
        self._validated_usb_sysfs_name = ''
        self._mode_switch_deadline = 0.0

    def _require_pairing(self, reason):
        """未保存绑定 ID 时保持禁止控制，并等待 application 完成采集。"""
        self.stage = ControllerValidationStage.PAIRING_REQUIRED
        self.binding_ok = False
        self.reason = reason
        self.bound_id = ''
        self.current = None
        self._validated_usb_path = ''
        self._validated_usb_sysfs_name = ''
        self._mode_switch_deadline = 0.0

    def _validate_xinput(self):
        """读取本次插入接收器的真实 ID，成功后等待用户切换 D 模式。"""
        try:
            self.bound_id, self.current = check_controller_binding(
                self.hardware_config_file, self.usb_sysfs)
        except ControllerEnumerationPendingError as exc:
            self.binding_ok = False
            self.reason = str(exc)
            if self.monotonic() < self._validation_deadline:
                return False
            if not self._enumeration_timeout_reported:
                self._enumeration_timeout_reported = True
                return True
            return False
        except ControllerBindingRequiredError as exc:
            self._require_pairing(str(exc))
            return True
        except Exception as exc:
            self._reject(str(exc))
            return True

        self._validated_usb_path = self.current.usb_path
        self._validated_usb_sysfs_name = self.current.usb_sysfs_name
        self._mode_switch_deadline = 0.0
        self.stage = ControllerValidationStage.WAITING_FOR_DINPUT
        self.binding_ok = False
        self.reason = (f'X 模式 ID 校验通过：{self.current.unique_id}；'
                       '请将手柄切换到 D 模式并短按、松开任意键唤醒；'
                       '设备消失后需在'
                       f'{self.mode_switch_timeout_s:.1f}秒内重新枚举')
        return True

    def _start_dinput_switch(self):
        """X模式消失后保留本次校验，在限时内等待同端口D模式。"""
        self.stage = ControllerValidationStage.SWITCHING_TO_DINPUT
        self.binding_ok = False
        self._mode_switch_deadline = (self.monotonic() +
                                      self.mode_switch_timeout_s)
        self.reason = ('X→D 模式切换中，USB接收器暂时消失；'
                       f'等待 {self.mode_switch_timeout_s:.1f}秒内在同一端口重新枚举')

    def _mode_switch_timed_out(self):
        """D模式未在限时内出现时撤销X模式会话校验。"""
        self._reset_for_xinput(
            f'X→D 模式切换超过 {self.mode_switch_timeout_s:.1f}秒，'
            '已清除本次X模式校验；请重新切到 X 模式校验 ID')

    def _wait_for_dinput_device(self, receiver):
        """确认 D 模式沿用同一 USB 端口，输入设备就绪后才放行。"""
        if (receiver.usb_path != self._validated_usb_path or
                receiver.usb_sysfs_name != self._validated_usb_sysfs_name):
            self._reject('D 模式接收器不在刚才通过 X 模式校验的 USB 端口，'
                         '请拔出后以 X 模式重新插入校验')
            return True

        # D模式USB已在切换窗口内出现；后续只等待joy输入设备，
        # 不再受X→D的2秒USB重新枚举窗口限制。
        self.stage = ControllerValidationStage.DINPUT_ENUMERATING
        self._mode_switch_deadline = 0.0

        try:
            _validate_joy_device(receiver, self.joy_dev, self.usb_sysfs,
                                 self.input_sysfs)
        except ControllerEnumerationPendingError as exc:
            self.binding_ok = False
            self.reason = str(exc)
            if self.monotonic() < self._validation_deadline:
                return False
            if not self._enumeration_timeout_reported:
                self._enumeration_timeout_reported = True
                return True
            return False

        self.stage = ControllerValidationStage.AUTHORIZED
        self.binding_ok = True
        self._mode_switch_deadline = 0.0
        self.current = ControllerIdentity(
            unique_id=self.bound_id,
            product=receiver.product,
            input_mode=F710_INPUT_MODES[receiver.product],
            serial_source='validated_xinput_session',
            identity_verified=False,
            usb_path=receiver.usb_path,
            usb_sysfs_name=receiver.usb_sysfs_name,
        )
        self.reason = ''
        return True

    def poll(self):
        """轮询状态；返回 True 表示产生了需要对外报告的新事件。"""
        receiver_instances = scan_f710_receiver_instances(self.usb_sysfs)
        receiver_changed = receiver_instances != self._receiver_instances

        if len(receiver_instances) > 1:
            self._receiver_instances = receiver_instances
            if self.stage != ControllerValidationStage.REJECTED or \
                    receiver_changed:
                self._reject('检测到多个 F710 接收器，禁止手柄控制')
                return True
            return False

        if not receiver_instances:
            had_receiver = bool(self._receiver_instances)
            self._receiver_instances = ()
            if self.stage == ControllerValidationStage.WAITING_FOR_DINPUT:
                self._start_dinput_switch()
                return True
            if self.stage == ControllerValidationStage.SWITCHING_TO_DINPUT:
                if self.monotonic() >= self._mode_switch_deadline:
                    self._mode_switch_timed_out()
                    return True
                return False
            if had_receiver:
                self._reset_for_xinput('手柄 USB 接收器已拔出；重新插入后必须先用 X 模式校验 ID')
                return True
            return False

        receiver = receiver_instances[0]
        self._receiver_instances = receiver_instances

        if receiver.product == F710_PRODUCT_XINPUT:
            if (not receiver_changed and
                    self.stage in (ControllerValidationStage.WAITING_FOR_DINPUT,
                                   ControllerValidationStage.PAIRING_REQUIRED,
                                   ControllerValidationStage.REJECTED)):
                return False
            self.binding_ok = False
            if receiver_changed:
                self._validation_deadline = (self.monotonic() +
                                             self.enumeration_timeout_s)
                self._enumeration_timeout_reported = False
            return self._validate_xinput()

        # D模式只能承接同一次插入周期内已经成功的X模式校验。
        if self.stage in (ControllerValidationStage.WAITING_FOR_DINPUT,
                          ControllerValidationStage.SWITCHING_TO_DINPUT,
                          ControllerValidationStage.DINPUT_ENUMERATING):
            if (self.stage == ControllerValidationStage.SWITCHING_TO_DINPUT and
                    self.monotonic() >= self._mode_switch_deadline):
                self._mode_switch_timed_out()
                return True
            if receiver_changed:
                self._validation_deadline = (self.monotonic() +
                                             self.enumeration_timeout_s)
                self._enumeration_timeout_reported = False
            return self._wait_for_dinput_device(receiver)

        if self.stage == ControllerValidationStage.AUTHORIZED:
            if receiver_changed:
                self._reset_for_xinput('D 模式接收器实例发生变化，旧授权已撤销；'
                                       '请先切换到 X 模式重新校验 ID')
                return True
            return False

        if (receiver_changed or
                self.stage != ControllerValidationStage.WAITING_FOR_XINPUT):
            self._reset_for_xinput('检测到 D 模式手柄，但本次插入尚未完成 X 模式 ID 校验；'
                                   '请先切换到 X 模式')
            return True
        return False

    @property
    def validation_pending(self):
        """是否仍在等待 X 校验或等待切换、枚举 D 模式。"""
        return self.stage in (
            ControllerValidationStage.WAITING_FOR_XINPUT,
            ControllerValidationStage.WAITING_FOR_DINPUT,
            ControllerValidationStage.SWITCHING_TO_DINPUT,
            ControllerValidationStage.DINPUT_ENUMERATING,
        )

    @property
    def pairing_required(self):
        """是否需要 application 采集并写入手柄绑定 ID。"""
        return self.stage == ControllerValidationStage.PAIRING_REQUIRED
