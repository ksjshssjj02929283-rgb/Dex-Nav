"""各子命令共用的小工具：输出目录、表格、CSV、状态码含义、按发布者匹配订阅 QoS。"""
import csv
import glob
import os
import re
import time

_CODE_CACHE = None


def default_out(prefix):
    """默认输出目录 ~/nav_eval/<前缀>_<时间>。"""
    return os.path.expanduser(time.strftime(f"~/nav_eval/{prefix}_%Y%m%d_%H%M%S"))


def now_text(t=None):
    t = time.time() if t is None else t
    return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(t)) + f".{int((t % 1) * 1000):03d}"


def md_table(headers, rows):
    def cell(v):
        return "" if v is None else str(v).replace("|", "\\|").replace("\n", " ")
    lines = ["| " + " | ".join(headers) + " |", "|" + "---|" * len(headers)]
    lines += ["| " + " | ".join(cell(v) for v in r) + " |" for r in rows]
    return "\n".join(lines)


def write_csv(path, headers, rows, append=False):
    new = not (append and os.path.exists(path))
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "a" if append else "w", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(headers)
        w.writerows(rows)


def _share_dir(pkg):
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory(pkg)
    except Exception:  # noqa: BLE001  没有 ament 环境时不提供含义
        return None


def load_codes(pkg="nav_interfaces"):
    """从已安装的 .msg / .srv / .action 里读出 7 位状态码的常量名和行尾注释，返回 {码: (常量名, 说明)}。"""
    global _CODE_CACHE
    if _CODE_CACHE is not None:
        return _CODE_CACHE
    codes = {}
    share = _share_dir(pkg)
    if share:
        pat = re.compile(r"^\s*int32\s+([A-Z][A-Z0-9_]*)\s*=\s*(\d{7})\b(?:\s*#\s*(.*))?$")
        for path in sorted(glob.glob(os.path.join(share, "*", "*.*"))):
            if not path.endswith((".msg", ".srv", ".action")):
                continue
            try:
                with open(path, encoding="utf-8") as f:
                    for line in f:
                        m = pat.match(line)
                        if m and int(m.group(2)) not in codes:
                            codes[int(m.group(2))] = (m.group(1), (m.group(3) or "").strip())
            except OSError:
                continue
    _CODE_CACHE = codes
    return codes


def code_text(code):
    """状态码转成「码 常量名（说明）」；0 返回「0」。"""
    try:
        code = int(code)
    except (TypeError, ValueError):
        return str(code)
    if code == 0:
        return "0"
    name, note = load_codes().get(code, ("", ""))
    if not name:
        return str(code)
    return f"{code} {name}" + (f"（{note}）" if note else "")


def constant_name(msg_cls, prefix_filter, value):
    """在消息类的常量里找值为 value 的名字（如 NavigationState 的状态名）。"""
    for k in dir(msg_cls):
        if k.isupper() and not k.startswith("_") and (not prefix_filter or prefix_filter(k)):
            try:
                if getattr(msg_cls, k) == value:
                    return k
            except Exception:  # noqa: BLE001
                continue
    return str(value)


def qos_for(node, topic, depth=10):
    """按话题当前发布者的 QoS 选订阅 QoS：发布者都是 transient_local 时用 transient_local（能收到锁存的最新值），
    否则用 volatile（和 volatile 发布者兼容）；可靠性同理。没有发布者时用 reliable + volatile。"""
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    infos = node.get_publishers_info_by_topic(topic)
    durability = DurabilityPolicy.VOLATILE
    reliability = ReliabilityPolicy.RELIABLE
    if infos:
        if all(i.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL for i in infos):
            durability = DurabilityPolicy.TRANSIENT_LOCAL
        if any(i.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT for i in infos):
            reliability = ReliabilityPolicy.BEST_EFFORT
    return QoSProfile(depth=depth, durability=durability, reliability=reliability)


def import_msg(type_str):
    """'nav_interfaces/msg/NavigationState' → 消息类；导入失败返回 None。"""
    try:
        from rosidl_runtime_py.utilities import get_message
        return get_message(type_str)
    except (AttributeError, ModuleNotFoundError, ValueError):
        return None
