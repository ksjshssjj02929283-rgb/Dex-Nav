"""读写其他节点参数（rcl_interfaces get_parameters / set_parameters），以及路网速度链的关键参数清单。"""
import time

FOLLOWER = "/path_follower_node"
SMOOTHER = "/velocity_smoother_path"
SPEED_PARAMS = {
    FOLLOWER: ["max_linear_velocity", "max_linear_acceleration", "max_linear_deceleration", "max_linear_jerk"],
    SMOOTHER: ["max_velocity", "min_velocity", "max_accel", "max_decel"],
}


def from_value(v):
    """rcl_interfaces/ParameterValue → Python 值；未设置返回 None。"""
    t = v.type
    return {1: lambda: v.bool_value, 2: lambda: v.integer_value, 3: lambda: v.double_value,
            4: lambda: v.string_value, 6: lambda: list(v.bool_array_value),
            7: lambda: list(v.integer_array_value), 8: lambda: list(v.double_array_value),
            9: lambda: list(v.string_array_value)}.get(t, lambda: None)()


def to_value(x):
    from rcl_interfaces.msg import ParameterType, ParameterValue
    v = ParameterValue()
    if isinstance(x, bool):
        v.type, v.bool_value = ParameterType.PARAMETER_BOOL, x
    elif isinstance(x, int):
        v.type, v.integer_value = ParameterType.PARAMETER_INTEGER, x
    elif isinstance(x, float):
        v.type, v.double_value = ParameterType.PARAMETER_DOUBLE, x
    elif isinstance(x, str):
        v.type, v.string_value = ParameterType.PARAMETER_STRING, x
    elif isinstance(x, (list, tuple)) and all(isinstance(e, (int, float)) and not isinstance(e, bool) for e in x):
        v.type, v.double_array_value = ParameterType.PARAMETER_DOUBLE_ARRAY, [float(e) for e in x]
    else:
        raise ValueError(f"不支持的参数值 {x!r}")
    return v


def _call(node, cli, req, timeout):
    import rclpy
    fut = cli.call_async(req)
    end = time.time() + timeout
    while not fut.done() and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    return fut.result() if fut.done() else None


def get_params(node, target, names, timeout=3.0):
    """返回 {名: 值}；节点不在线返回 None。"""
    from rcl_interfaces.srv import GetParameters
    cli = node.create_client(GetParameters, f"{target}/get_parameters")
    try:
        if not cli.wait_for_service(timeout_sec=timeout):
            return None
        r = _call(node, cli, GetParameters.Request(names=list(names)), timeout)
        if r is None or len(r.values) != len(names):
            return None
        return {n: from_value(v) for n, v in zip(names, r.values)}
    finally:
        node.destroy_client(cli)


def set_params(node, target, values, timeout=3.0):
    """values {名: 值} → [(名, 成功, 原因)]；节点不在线返回 None。"""
    from rcl_interfaces.msg import Parameter
    from rcl_interfaces.srv import SetParameters
    cli = node.create_client(SetParameters, f"{target}/set_parameters")
    try:
        if not cli.wait_for_service(timeout_sec=timeout):
            return None
        names = list(values)
        req = SetParameters.Request(parameters=[Parameter(name=n, value=to_value(values[n])) for n in names])
        r = _call(node, cli, req, timeout)
        if r is None:
            return None
        return [(n, res.successful, res.reason) for n, res in zip(names, r.results)]
    finally:
        node.destroy_client(cli)


def snapshot(node, timeout=3.0):
    """速度链参数快照 {节点: {名: 值} 或 None}。"""
    return {n: get_params(node, n, names, timeout) for n, names in SPEED_PARAMS.items()}
