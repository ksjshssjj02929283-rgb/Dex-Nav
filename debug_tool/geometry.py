"""仿真器与绘图共用的二维几何计算。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

import numpy as np


@dataclass(frozen=True)
class PathProjection:
    point: np.ndarray
    segment: int
    ratio: float
    distance: float
    arc_length: float


def cumulative_lengths(path: np.ndarray) -> np.ndarray:
    if len(path) == 0:
        return np.zeros(0)
    delta = np.diff(path, axis=0)
    return np.concatenate(([0.0], np.cumsum(np.linalg.norm(delta, axis=1))))


def project_to_path(path: np.ndarray, point: np.ndarray) -> PathProjection:
    if len(path) == 1:
        distance = float(np.linalg.norm(point - path[0]))
        return PathProjection(path[0].copy(), 0, 0.0, distance, 0.0)
    starts = path[:-1]
    vectors = path[1:] - starts
    lengths_sq = np.einsum("ij,ij->i", vectors, vectors)
    ratios = np.zeros(len(vectors))
    valid = lengths_sq > 1e-12
    ratios[valid] = np.einsum("ij,ij->i", point - starts, vectors)[valid] / lengths_sq[valid]
    ratios = np.clip(ratios, 0.0, 1.0)
    projected = starts + ratios[:, None] * vectors
    distances_sq = np.einsum("ij,ij->i", projected - point, projected - point)
    segment = int(np.argmin(distances_sq))
    cumulative = cumulative_lengths(path)
    segment_length = float(np.linalg.norm(vectors[segment]))
    arc = float(cumulative[segment] + ratios[segment] * segment_length)
    return PathProjection(
        projected[segment], segment, float(ratios[segment]),
        float(np.sqrt(distances_sq[segment])), arc,
    )


def point_at_arc(path: np.ndarray, arc: float) -> np.ndarray:
    lengths = cumulative_lengths(path)
    if len(path) == 0:
        return np.zeros(2)
    if arc <= 0.0:
        return path[0].copy()
    if arc >= lengths[-1]:
        return path[-1].copy()
    upper = int(np.searchsorted(lengths, arc, side="right"))
    lower = upper - 1
    span = lengths[upper] - lengths[lower]
    ratio = 0.0 if span <= 1e-12 else (arc - lengths[lower]) / span
    return path[lower] + ratio * (path[upper] - path[lower])


def carrot_point(path: np.ndarray, robot_xy: np.ndarray, lookahead: float) -> np.ndarray:
    """返回机器人投影点之后，第一个精确的路径线段与前瞻圆交点。"""
    projection = project_to_path(path, robot_xy)
    first_segment = projection.segment
    segment_start = projection.point
    radius_sq = lookahead * lookahead
    for segment in range(first_segment, len(path) - 1):
        p1 = segment_start if segment == first_segment else path[segment]
        p2 = path[segment + 1]
        delta = p2 - p1
        a = float(np.dot(delta, delta))
        if a <= 1e-12:
            continue
        offset = p1 - robot_xy
        b = 2.0 * float(np.dot(offset, delta))
        c = float(np.dot(offset, offset) - radius_sq)
        discriminant = b * b - 4.0 * a * c
        if discriminant < 0.0:
            continue
        root = np.sqrt(max(discriminant, 0.0))
        candidates = sorted(((-b - root) / (2.0 * a), (-b + root) / (2.0 * a)))
        valid = [value for value in candidates if -1e-9 <= value <= 1.0 + 1e-9]
        if valid:
            return p1 + np.clip(max(valid), 0.0, 1.0) * delta
    return path[-1].copy()


def to_robot_frame(point: np.ndarray, pose: Tuple[float, float, float]) -> np.ndarray:
    x, y, yaw = pose
    dx, dy = point[0] - x, point[1] - y
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([c * dx + s * dy, -s * dx + c * dy])


def wrap_angle(angle: float) -> float:
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


def path_curvature(path: np.ndarray, sample_distance: float) -> np.ndarray:
    """在目标点前后约 sample_distance 的位置进行三点曲率采样。"""
    if len(path) < 3:
        return np.zeros(len(path))
    lengths = cumulative_lengths(path)
    result = np.zeros(len(path))
    for i, arc in enumerate(lengths):
        before = point_at_arc(path, max(0.0, arc - sample_distance))
        middle = path[i]
        after = point_at_arc(path, min(lengths[-1], arc + sample_distance))
        ab, bc, ca = middle - before, after - middle, before - after
        denominator = np.linalg.norm(ab) * np.linalg.norm(bc) * np.linalg.norm(ca)
        if denominator <= 1e-10:
            continue
        cross = ab[0] * bc[1] - ab[1] * bc[0]
        result[i] = 2.0 * cross / denominator
    return result
