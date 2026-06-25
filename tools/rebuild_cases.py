#!/usr/bin/env python3
"""Rebuild all manifest-managed payload cases through the general pipeline."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import secrets
import shutil
from types import SimpleNamespace
from typing import Any

from generate_case import generate
from verify_cases import verify_case


DEMO_ROOT = Path(__file__).resolve().parents[1]
DATA_DIR = DEMO_ROOT / "data"
PACKAGE_DATA_DIR = DEMO_ROOT / "src" / "swarm_random_payload" / "data"
DEFAULT_MANIFEST = DATA_DIR / "case_manifest.json"
DEFAULT_PAPER_QP_CONFIG = {
    "timeStep": 0.02,
    "fixedTimeBound": 12.0,
    "uMaxMetersPerSecond": 10.0,
    "delta1LinearWeight": 0.0,
    "constraintTolerance": 0.001,
    "targetTolerance": 1.3,
    "enforceDiscreteSafety": True,
}


def read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def copy_if_different(source: Path, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    if source.resolve() != destination.resolve():
        shutil.copyfile(source, destination)


def apply_source_options(path: Path, options: dict[str, Any]) -> None:
    payload = read_json(path)
    constants = payload.get("constants", {})
    for key, source_key in (
        ("formationScale", "formationScale"),
        ("safeDistance", "safeDistance"),
        ("safetyMargin", "safetyMargin"),
        ("obstacleMargin", "obstacleMargin"),
    ):
        if key not in payload and isinstance(constants, dict) and source_key in constants:
            payload[key] = constants[source_key]

    if options.get("source_regions_only"):
        payload["sourceRegionsOnly"] = True
    if options.get("enable_single_file_bridges"):
        payload["enableSingleFileBridges"] = True
    if options.get("insert_interior_states"):
        payload["insertInteriorStates"] = True
    if "task_placement_tolerances" in options:
        payload["taskPlacementTolerance"] = options["task_placement_tolerances"]
    if "allow_return_repair" in options:
        payload["allowReturnRepair"] = bool(options["allow_return_repair"])

    explicit_source_keys = {
        "formation_scale": "formationScale",
        "safe_distance": "safeDistance",
        "safety_margin": "safetyMargin",
        "obstacle_margin": "obstacleMargin",
    }
    for option_key, source_key in explicit_source_keys.items():
        if option_key in options:
            payload[source_key] = options[option_key]

    write_json(path, payload)


def apply_case_options(case: dict[str, Any], options: dict[str, Any]) -> dict[str, Any]:
    if not options.get("paper_qp"):
        return case

    paper_qp = dict(DEFAULT_PAPER_QP_CONFIG)
    paper_qp.update(options.get("paper_qp_config", {}))
    case["schema"] = "swarm_random_payload_case.paper_qp.v1"
    case["control"] = {
        "mode": "paper_qp",
        "paper_qp": paper_qp,
    }
    return case


def resolve_path(raw: str) -> Path:
    path = Path(raw).expanduser()
    if path.is_absolute():
        return path
    return (DEMO_ROOT / path).resolve()


def merged_options(defaults: dict[str, Any], item: dict[str, Any]) -> dict[str, Any]:
    result = dict(defaults)
    result.update({k: v for k, v in item.items() if k not in {"name", "input"}})
    return result


def resolve_random_seed(raw_seed: Any) -> int:
    if isinstance(raw_seed, str) and raw_seed.lower() in {"auto", "random"}:
        return secrets.randbelow(2**31 - 1)
    return int(raw_seed)


def build_case(item: dict[str, Any], defaults: dict[str, Any], retries: int = 8) -> Path:
    name = item["name"]
    source_path = resolve_path(item["input"])
    options = merged_options(defaults, item)
    auto_seed = isinstance(options["random_seed"], str) and options["random_seed"].lower() in {"auto", "random"}
    attempts = max(1, retries if auto_seed else 1)

    map_out = DATA_DIR / f"{name}_map.json"
    package_map_out = PACKAGE_DATA_DIR / f"{name}_map.json"
    case_out = DATA_DIR / f"{name}_case.json"
    package_case_out = PACKAGE_DATA_DIR / f"{name}_case.json"

    copy_if_different(source_path, map_out)
    copy_if_different(source_path, package_map_out)
    for path in (map_out, package_map_out):
        apply_source_options(path, options)

    last_error: Exception | None = None
    for attempt in range(1, attempts + 1):
        random_seed = resolve_random_seed(options["random_seed"])
        attempt_label = f" attempt={attempt}/{attempts}" if attempts > 1 else ""
        print(f"[seed] {name}:{attempt_label} random_seed={random_seed}")

        args = SimpleNamespace(
            input=str(map_out),
            output=str(case_out),
            package_output=str(package_case_out),
            random_seed=random_seed,
            seed_budget=int(options["seed_budget"]),
            max_regions=int(options["max_regions"]),
            min_regions=int(options["min_regions"]),
            iteration_limit=int(options["iteration_limit"]),
            min_region_area=float(options["min_region_area"]),
            obstacle_overlap_tolerance=float(options["obstacle_overlap_tolerance"]),
            source_regions_only=bool(options.get("source_regions_only", False)),
        )
        try:
            case = generate(args)
        except Exception as exc:
            last_error = exc
            print(f"[retry] {name}: generation failed with seed {random_seed}: {exc}")
            continue

        case = apply_case_options(case, options)
        write_json(case_out, case)
        write_json(package_case_out, case)
        print(
            f"[built] {name}: regions={len(case['regions'])} "
            f"bridges={len(case['bridges'])} states={len(case['states'])}"
        )
        return case_out

    raise RuntimeError(f"failed to build {name} after {attempts} attempt(s)") from last_error


def copy_case_alias(source: str, target: str) -> Path:
    source_case = DATA_DIR / f"{source}_case.json"
    target_case = DATA_DIR / f"{target}_case.json"
    package_target_case = PACKAGE_DATA_DIR / f"{target}_case.json"
    copy_if_different(source_case, target_case)
    copy_if_different(source_case, package_target_case)
    print(f"[alias] {target}_case.json <- {source}_case.json")
    return target_case


def copy_map_alias(source: str, target: str) -> Path:
    source_map = DATA_DIR / f"{source}_map.json"
    target_map = DATA_DIR / f"{target}_map.json"
    package_target_map = PACKAGE_DATA_DIR / f"{target}_map.json"
    copy_if_different(source_map, target_map)
    copy_if_different(source_map, package_target_map)
    print(f"[alias] {target}_map.json <- {source}_map.json")
    return target_map


def copy_manifest(manifest_path: Path) -> None:
    copy_if_different(manifest_path, DATA_DIR / manifest_path.name)
    copy_if_different(manifest_path, PACKAGE_DATA_DIR / manifest_path.name)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", default=str(DEFAULT_MANIFEST))
    parser.add_argument(
        "--case",
        action="append",
        dest="cases",
        help="Build only one manifest case by name. Can be repeated.",
    )
    parser.add_argument("--retries", type=int, default=8, help="Retry count for random_seed=auto cases.")
    parser.add_argument("--no-verify", action="store_true")
    args = parser.parse_args()

    manifest_path = Path(args.manifest).expanduser().resolve()
    manifest = read_json(manifest_path)
    if manifest.get("schema") != "swarm_random_payload_case_manifest_v1":
        raise RuntimeError(f"unsupported manifest schema in {manifest_path}")
    copy_manifest(manifest_path)

    built_cases = []
    defaults = manifest.get("defaults", {})
    selected = set(args.cases or [])
    manifest_names = {item["name"] for item in manifest.get("cases", [])}
    unknown = sorted(selected - manifest_names)
    if unknown:
        raise RuntimeError(f"unknown manifest case(s): {', '.join(unknown)}")

    for item in manifest.get("cases", []):
        if selected and item["name"] not in selected:
            continue
        built_cases.append(build_case(item, defaults, retries=args.retries))

    for alias in manifest.get("case_aliases", []):
        if selected and alias["target"] not in selected:
            continue
        built_cases.append(copy_case_alias(alias["source"], alias["target"]))
    for alias in manifest.get("map_aliases", []):
        if selected and alias["target"] not in selected:
            continue
        copy_map_alias(alias["source"], alias["target"])

    if args.no_verify:
        return
    checked = built_cases if selected else sorted(DATA_DIR.glob("*_case.json"))
    failed = False
    for path in checked:
        errors = verify_case(path)
        if errors:
            failed = True
            print(f"[FAIL] {path.name}: {errors}")
        else:
            print(f"[OK] {path.name}")
    if failed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
