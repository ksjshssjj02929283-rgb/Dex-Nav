"""供界面与报告使用的独立速度 PID 阶跃响应。"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .config import PIDConfig
from .control import PIDState


@dataclass
class PIDResponse:
    time: np.ndarray
    target: np.ndarray
    command: np.ndarray
    measured: np.ndarray
    error: np.ndarray


def simulate_pid_step(
    config: PIDConfig,
    target_speed: float = 0.4,
    duration: float = 8.0,
    dt: float = 0.02,
    plant_tau: float = 0.25,
    delay: float = 0.08,
) -> PIDResponse:
    steps = int(duration / dt)
    times = np.arange(steps) * dt
    targets = np.where(times >= 1.0, target_speed, 0.0)
    pid = PIDState()
    measured = 0.0
    command = 0.0
    delay_steps = max(0, int(round(delay / dt)))
    queue = [0.0] * (delay_steps + 1)
    commands, measurements, errors = [], [], []
    for target in targets:
        error = target - measured
        acceleration = pid.step(error, dt, config)
        acceleration = float(np.clip(acceleration, -abs(config.max_accel), abs(config.max_accel)))
        command = float(np.clip(command + acceleration * dt, 0.0, max(target_speed, 0.0)))
        queue.append(command)
        delayed = queue.pop(0)
        alpha = 1.0 if plant_tau <= 0 else min(1.0, dt / plant_tau)
        measured += alpha * (delayed - measured)
        commands.append(command)
        measurements.append(measured)
        errors.append(error)
    return PIDResponse(
        times, targets, np.asarray(commands), np.asarray(measurements), np.asarray(errors)
    )
