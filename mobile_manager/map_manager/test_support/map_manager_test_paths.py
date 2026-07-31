"""Shared paths for the private map_manager test implementation."""

from __future__ import annotations

import os
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_MAPS_ROOT = Path(
    os.environ.get(
        "MAP_MANAGER_MAPS_ROOT",
        str(Path.home() / ".dexe/data/maps"),
    )
).expanduser()
DEFAULT_ROS_INSTALL_ROOT = Path(
    os.environ.get(
        "MAP_MANAGER_ROS_INSTALL_ROOT",
        str(Path.home() / "w1/install"),
    )
).expanduser()
DEFAULT_SOURCE_MAP_NAME = "map_standard"
DEFAULT_RESULT_ROOT = Path(
    os.environ.get(
        "MAP_MANAGER_TEST_RESULT_ROOT",
        str(Path.home() / ".dexe/navigation/map_manager_test"),
    )
).expanduser()


def runtime_manifest_path(maps_root: Path = DEFAULT_MAPS_ROOT) -> Path:
    override = os.environ.get("MAP_MANAGER_TEST_MANIFEST")
    if override:
        return Path(override).expanduser()
    return maps_root.parent / "map_manager_test/standard_map_manifest.json"


def runtime_state_path(maps_root: Path = DEFAULT_MAPS_ROOT) -> Path:
    return maps_root.parent / "map_manager_test/managed_maps.json"


def install_prefix_candidates() -> tuple[Path, ...]:
    candidates: list[Path] = []
    override = os.environ.get("MAP_MANAGER_INSTALL_PREFIX")
    if override:
        candidates.append(Path(override).expanduser())
    candidates.extend(SCRIPT_DIR.parents)
    candidates.extend(
        Path(value).expanduser()
        for value in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep)
        if value
    )
    candidates.extend(
        (
            DEFAULT_ROS_INSTALL_ROOT / "map_manager",
            DEFAULT_ROS_INSTALL_ROOT,
        )
    )
    return tuple(dict.fromkeys(path.resolve() for path in candidates))


def installed_package_prefix() -> Path | None:
    for prefix in install_prefix_candidates():
        package_share = prefix / "share/map_manager"
        if (package_share / "package.xml").is_file():
            return prefix
    return None


def installed_package_share() -> Path | None:
    prefix = installed_package_prefix()
    return prefix / "share/map_manager" if prefix is not None else None


def installed_manifest_path() -> Path:
    package_share = installed_package_share()
    if package_share is not None:
        return package_share / "test_data/standard_map_manifest.json"
    # Source-tree fallback: <package>/test_support/<module>.
    return SCRIPT_DIR.parent / "test_data/standard_map_manifest.json"


def default_manifest_path(maps_root: Path = DEFAULT_MAPS_ROOT) -> Path:
    candidates = (
        runtime_manifest_path(maps_root),
        installed_manifest_path(),
    )
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    return candidates[0]


def detect_source_package() -> Path | None:
    override = os.environ.get("MAP_MANAGER_SOURCE_PACKAGE")
    candidates = [
        Path(override).expanduser() if override else None,
        SCRIPT_DIR.parent if (SCRIPT_DIR.parent / "CMakeLists.txt").is_file() else None,
    ]
    for candidate in candidates:
        if candidate is not None and (candidate / "package.xml").is_file():
            return candidate
    return None


def packaged_standard_map_path() -> Path:
    override = os.environ.get("MAP_MANAGER_PACKAGED_STANDARD_MAP")
    source_package = detect_source_package()
    package_share = installed_package_share()
    candidates = [
        Path(override).expanduser() if override else None,
        (
            package_share / "test_data" / DEFAULT_SOURCE_MAP_NAME
            if package_share is not None
            else None
        ),
        (
            source_package / "test_data" / DEFAULT_SOURCE_MAP_NAME
            if source_package is not None
            else None
        ),
    ]
    for candidate in candidates:
        if candidate is not None and candidate.is_dir():
            return candidate
    return (
        DEFAULT_ROS_INSTALL_ROOT
        / "share/map_manager/test_data"
        / DEFAULT_SOURCE_MAP_NAME
    )
