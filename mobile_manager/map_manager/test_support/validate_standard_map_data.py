#!/usr/bin/env python3
"""Private fixture validator used by the map_manager test entry."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass
class Finding:
    status: str
    item: str
    detail: str


class Validator:
    def __init__(
        self,
        dataset: Path,
        *,
        maps_root: Path | None = None,
        manifest_path: Path | None = None,
    ) -> None:
        self.dataset = dataset.resolve()
        self.maps_root = (
            maps_root.resolve() if maps_root is not None else self.dataset / "maps"
        )
        self.manifest_path = (
            manifest_path.resolve()
            if manifest_path is not None
            else self.dataset / "manifest.json"
        )
        self.findings: list[Finding] = []

    def add(self, status: str, item: str, detail: str) -> None:
        self.findings.append(Finding(status, item, detail))

    def check(self, condition: bool, item: str, detail: str) -> None:
        self.add("PASS" if condition else "FAIL", item, detail)

    def load_json(self, path: Path) -> dict[str, Any]:
        try:
            value = json.loads(path.read_text(encoding="utf-8"))
        except Exception as exc:
            self.add("FAIL", str(path), f"invalid JSON: {exc}")
            return {}
        if not isinstance(value, dict):
            self.add("FAIL", str(path), "top-level JSON must be an object")
            return {}
        return value

    @staticmethod
    def sha256(path: Path) -> str:
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
        return digest.hexdigest()

    @staticmethod
    def image_from_yaml(path: Path) -> str:
        if not path.is_file():
            return ""
        for line in path.read_text(encoding="utf-8").splitlines():
            match = re.match(r"^\s*image\s*:\s*[\"']?([^\"'#]+)", line)
            if match:
                return match.group(1).strip()
        return ""

    def require_files(self, map_dir: Path, relative_paths: list[str], role: str) -> None:
        for relative in relative_paths:
            path = map_dir / relative
            self.check(path.is_file(), f"{role}:{relative}", str(path))
            if path.is_file() and path.stat().st_size == 0:
                self.add("FAIL", f"{role}:{relative}", "file is empty")
            if (
                path.is_file()
                and path.suffix == ".pbstream"
                and path.stat().st_size < 1024
            ):
                self.add(
                    "FAIL",
                    f"{role}:{relative}",
                    f"pbstream is suspiciously small: {path.stat().st_size} bytes",
                )

    def validate_assets(
        self,
        path: Path,
        role: str,
        expected_counts: dict[str, int],
        expected_canaries: list[str] | None = None,
    ) -> None:
        data = self.load_json(path) if path.is_file() else {}
        actual_names: set[str] = set()
        for key in ("points", "routes", "areas"):
            value = data.get(key)
            self.check(
                isinstance(value, list),
                f"{role}:assets.{key}",
                f"type={type(value).__name__}",
            )
            if isinstance(value, list) and key in expected_counts:
                self.check(
                    len(value) == expected_counts[key],
                    f"{role}:assets.{key}.count",
                    f"actual={len(value)}, expected={expected_counts[key]}",
                )
            if isinstance(value, list):
                actual_names.update(
                    str(item.get("name", ""))
                    for item in value
                    if isinstance(item, dict)
                )
        if expected_canaries is not None:
            expected = set(expected_canaries)
            self.check(
                expected <= actual_names,
                f"{role}:assets.canaries",
                f"actual={sorted(actual_names)}, expected={sorted(expected)}",
            )

    def validate_layered(self, spec: dict[str, Any], role: str) -> None:
        name = str(spec.get("name", ""))
        map_dir = self.maps_root / name
        self.check(bool(name), f"{role}:name", name)
        self.check(map_dir.is_dir(), f"{role}:directory", str(map_dir))
        required = [
            "slam/map.yaml",
            "slam/map.pbstream",
            "assets/map_assets.json",
        ]
        self.require_files(map_dir, required, role)
        image = self.image_from_yaml(map_dir / "slam/map.yaml")
        self.check(bool(image), f"{role}:map.yaml.image", image or "missing")
        if image:
            self.require_files(map_dir, [f"slam/{image}"], role)
        self.validate_assets(
            map_dir / "assets/map_assets.json",
            role,
            spec.get("asset_counts", {}),
            spec.get("canaries", []),
        )

    def validate_legacy(self, spec: dict[str, Any]) -> None:
        role = "legacy_flat"
        name = str(spec.get("name", ""))
        map_dir = self.maps_root / name
        self.check(map_dir.is_dir(), f"{role}:directory", str(map_dir))
        self.require_files(
            map_dir,
            ["map.yaml", "map.pbstream", "map_assets.json"],
            role,
        )
        image = self.image_from_yaml(map_dir / "map.yaml")
        self.check(bool(image), f"{role}:map.yaml.image", image or "missing")
        if image:
            self.require_files(map_dir, [image], role)
        self.check(
            not (map_dir / "slam").exists() and not (map_dir / "assets").exists(),
            f"{role}:flat-layout",
            "slam/ and assets/ must not exist before migration",
        )
        unexpected = sorted(
            p.name
            for p in map_dir.iterdir()
            if p.is_file()
            and p.name not in {"map.yaml", image, "map.pbstream", "map_assets.json"}
        ) if map_dir.is_dir() else []
        self.check(
            not unexpected,
            f"{role}:no-extra-root-files",
            f"unexpected={unexpected}",
        )
        self.validate_assets(
            map_dir / "map_assets.json",
            role,
            spec.get("asset_counts", {}),
            spec.get("canaries", []),
        )

    def validate_broken(self, spec: dict[str, Any]) -> None:
        role = "broken_legacy"
        name = str(spec.get("name", ""))
        map_dir = self.maps_root / name
        self.check(map_dir.is_dir(), f"{role}:directory", str(map_dir))
        self.check(
            not (map_dir / "map.yaml").exists()
            and not (map_dir / "slam/map.yaml").exists(),
            f"{role}:missing-map-yaml",
            "both root and layered map.yaml must be absent",
        )
        self.require_files(map_dir, ["map.png", "map.pbstream"], role)

    def validate_rollback(self, spec: dict[str, Any]) -> None:
        role = "map_a.rollback"
        name = str(spec.get("name", ""))
        map_dir = self.maps_root / name
        rollback = spec.get("rollback", {})
        map_ts = str(rollback.get("map_version_ts", ""))
        assets_ts = str(rollback.get("assets_version_ts", ""))
        self.check(bool(map_ts), f"{role}:map_version_ts", map_ts)
        self.check(bool(assets_ts), f"{role}:assets_version_ts", assets_ts)
        if map_ts:
            self.require_files(
                map_dir,
                [
                    f"slam/backups/backup.{map_ts}/map.yaml",
                    f"slam/backups/backup.{map_ts}/map.png",
                    f"slam/backups/backup.{map_ts}/map.pbstream",
                ],
                role,
            )
        if assets_ts:
            backup = (
                map_dir
                / f"assets/backups/map_assets.{assets_ts}.json"
            )
            self.require_files(
                map_dir,
                [f"assets/backups/map_assets.{assets_ts}.json"],
                role,
            )
            self.validate_assets(
                backup,
                role,
                rollback.get("asset_counts", {}),
                rollback.get("canaries", []),
            )

    def validate_checksums(self, manifest: dict[str, Any]) -> None:
        checksums = manifest.get("sha256", {})
        if not checksums:
            self.add(
                "PENDING",
                "sha256",
                "manifest has no frozen checksums; fill them after data is finalized",
            )
            return
        for relative, expected in checksums.items():
            relative_path = Path(relative)
            if relative_path.parts and relative_path.parts[0] == "maps":
                path = self.maps_root.joinpath(*relative_path.parts[1:])
            else:
                path = self.dataset / relative_path
            if not path.is_file():
                self.add("FAIL", f"sha256:{relative}", "file missing")
                continue
            actual = self.sha256(path)
            if not expected or str(expected).startswith("__"):
                self.add("PENDING", f"sha256:{relative}", f"actual={actual}")
            else:
                self.check(
                    actual == expected,
                    f"sha256:{relative}",
                    f"actual={actual}, expected={expected}",
                )

    def validate_distinct_content(self, samples: dict[str, Any]) -> None:
        a_dir = self.maps_root / str(samples.get("map_a", {}).get("name", ""))
        b_dir = self.maps_root / str(samples.get("map_b", {}).get("name", ""))
        a_image_name = self.image_from_yaml(a_dir / "slam/map.yaml")
        b_image_name = self.image_from_yaml(b_dir / "slam/map.yaml")
        pairs = [
            (
                "map_a versus map_b image",
                a_dir / "slam" / a_image_name,
                b_dir / "slam" / b_image_name,
            ),
            (
                "map_a versus map_b assets",
                a_dir / "assets/map_assets.json",
                b_dir / "assets/map_assets.json",
            ),
        ]
        rollback = samples.get("map_a", {}).get("rollback", {})
        map_ts = str(rollback.get("map_version_ts", ""))
        assets_ts = str(rollback.get("assets_version_ts", ""))
        pairs.extend(
            [
                (
                    "map_a current versus rollback image",
                    a_dir / "slam" / a_image_name,
                    a_dir / f"slam/backups/backup.{map_ts}/map.png",
                ),
                (
                    "map_a current versus rollback assets",
                    a_dir / "assets/map_assets.json",
                    a_dir / f"assets/backups/map_assets.{assets_ts}.json",
                ),
            ]
        )
        for item, left, right in pairs:
            if not left.is_file() or not right.is_file():
                self.add(
                    "FAIL",
                    item,
                    f"cannot compare missing file: {left} or {right}",
                )
                continue
            left_sha = self.sha256(left)
            right_sha = self.sha256(right)
            self.check(
                left_sha != right_sha,
                item,
                f"left={left_sha}, right={right_sha}",
            )

    def run(self) -> dict[str, Any]:
        manifest_path = self.manifest_path
        self.check(manifest_path.is_file(), "manifest", str(manifest_path))
        manifest = self.load_json(manifest_path) if manifest_path.is_file() else {}
        self.check(
            manifest.get("schema_version") == 1,
            "schema_version",
            f"actual={manifest.get('schema_version')}",
        )
        samples = manifest.get("samples", {})
        required_roles = {"map_a", "map_b", "legacy_flat", "broken_legacy"}
        self.check(
            required_roles <= samples.keys(),
            "sample roles",
            f"actual={sorted(samples.keys())}",
        )

        names = [str(samples.get(role, {}).get("name", "")) for role in required_roles]
        self.check(
            all(name.startswith("mm_test_") for name in names),
            "safe name prefix",
            f"names={sorted(names)}",
        )
        self.check(
            len(set(names)) == len(names),
            "unique sample names",
            f"names={sorted(names)}",
        )

        for role in ("map_a", "map_b"):
            self.validate_layered(samples.get(role, {}), role)
        self.validate_legacy(samples.get("legacy_flat", {}))
        self.validate_broken(samples.get("broken_legacy", {}))
        self.validate_rollback(samples.get("map_a", {}))
        self.validate_distinct_content(samples)
        self.validate_checksums(manifest)

        summary = {
            "pass": sum(f.status == "PASS" for f in self.findings),
            "fail": sum(f.status == "FAIL" for f in self.findings),
            "pending": sum(f.status == "PENDING" for f in self.findings),
        }
        return {
            "dataset": str(self.dataset),
            "maps_root": str(self.maps_root),
            "manifest": str(self.manifest_path),
            "summary": summary,
            "findings": [f.__dict__ for f in self.findings],
        }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset", type=Path, nargs="?", default=Path("."))
    parser.add_argument("--maps-root", type=Path)
    parser.add_argument("--manifest", type=Path)
    parser.add_argument("--json-output", type=Path)
    parser.add_argument(
        "--console-flow",
        action="store_true",
        help="print every validation item as a readable execution flow",
    )
    args = parser.parse_args()

    result = Validator(
        args.dataset,
        maps_root=args.maps_root,
        manifest_path=args.manifest,
    ).run()
    output = json.dumps(result, ensure_ascii=False, indent=2)
    if args.console_flow:
        print("\n╭─ [FIXTURE] 冻结测试数据完整性")
        for index, finding in enumerate(result["findings"], 1):
            marker = "✓" if finding["status"] == "PASS" else "✗"
            print(
                f"│ {index:02d} {marker} CHECK [{finding['status']}] "
                f"{finding['item']} — {finding['detail']}"
            )
        summary = result["summary"]
        status = "PASS" if summary["fail"] == 0 else "FAIL"
        print(
            f"╰─ RESULT [{status}] PASS={summary['pass']} "
            f"FAIL={summary['fail']} PENDING={summary['pending']}"
        )
    else:
        print(output)
    if args.json_output:
        args.json_output.write_text(output + "\n", encoding="utf-8")
    return 0 if result["summary"]["fail"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
