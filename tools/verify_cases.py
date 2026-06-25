#!/usr/bin/env python3
"""Verify generated payload cases against the general concise pipeline rules."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any


DEMO_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_SRC = DEMO_ROOT / "src" / "swarm_random_payload"
sys.path.insert(0, str(PACKAGE_SRC))

from swarm_random_payload.random_payload_model import PayloadTransportScenario  # noqa: E402


REDUNDANT_SWITCH_STATES = {"s_drop_line", "s_home_line"}


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def paired_map_path(case_path: Path) -> Path | None:
    name = case_path.name
    if not name.endswith("_case.json"):
        return None
    candidate = case_path.with_name(name.replace("_case.json", "_map.json"))
    return candidate if candidate.exists() else None


def source_region_ids(path: Path | None) -> set[int]:
    if path is None:
        return set()
    data = load_json(path)
    ids = set()
    for region in data.get("regions", []):
        if isinstance(region, dict) and "id" in region:
            ids.add(int(region["id"]))
    return ids


def source_region_seed_by_id(path: Path | None) -> dict[int, tuple[float, float]]:
    if path is None:
        return {}
    data = load_json(path)
    result: dict[int, tuple[float, float]] = {}
    for region in data.get("regions", []):
        if not isinstance(region, dict) or "id" not in region:
            continue
        seed = region.get("seed")
        if isinstance(seed, dict) and "x" in seed and "y" in seed:
            result[int(region["id"])] = (float(seed["x"]), float(seed["y"]))
    return result


def close_point(a: tuple[float, float], b: tuple[float, float], tolerance: float = 1e-3) -> bool:
    return abs(a[0] - b[0]) <= tolerance and abs(a[1] - b[1]) <= tolerance


def verify_case(path: Path, strict_concise: bool = True) -> list[str]:
    errors: list[str] = []
    scenario = PayloadTransportScenario(case_file=str(path))
    failed = [name for name, ok in scenario.checks.items() if not ok]
    if failed:
        errors.append(f"runtime checks failed: {failed}")
        errors.append(
            "workspace clearance "
            f"{scenario.min_workspace_clearance():.3f}px < required {scenario.workspace_robot_margin:.3f}px"
            if "workspace_safe_execution" in failed
            else f"workspace clearance {scenario.min_workspace_clearance():.3f}px"
        )

    state_ids = [state.state_id for state in scenario.states]
    redundant = sorted(REDUNDANT_SWITCH_STATES.intersection(state_ids))
    if strict_concise and redundant:
        errors.append(f"redundant switch states present: {redundant}")

    if not scenario.case.get("route_region_paths"):
        errors.append("missing route_region_paths")
    if not scenario.case.get("states"):
        errors.append("missing states")
    if not scenario.case.get("bridges"):
        errors.append("missing bridges")

    loaded = [state for state in scenario.states if state.task_mode == "loaded"]
    delivered = [state for state in scenario.states if state.task_mode == "delivered"]
    if any(state.formation != "triangle" for state in loaded):
        errors.append("loaded states are not all triangle")
    if any(state.formation != "line" for state in delivered):
        errors.append("delivered states are not all line")
    if scenario.states[-1].formation != "triangle":
        errors.append("final state is not triangle")

    map_ids = source_region_ids(paired_map_path(path))
    if map_ids and load_json(path).get("sampling", {}).get("strategy", []) == ["source region seeds only"]:
        case_ids = {region["id"] for region in scenario.regions}
        if not case_ids.issubset(map_ids):
            errors.append(f"case ids {sorted(case_ids)} are not a subset of source ids {sorted(map_ids)}")

    source_seeds = source_region_seed_by_id(paired_map_path(path))
    for region in scenario.regions:
        source_seed = source_seeds.get(region["id"])
        if source_seed is None:
            continue
        if not close_point(region["seed"], source_seed):
            errors.append(
                f"{region['name']} reuses source id {region['id']} with different seed "
                f"{region['seed']} != {source_seed}"
            )

    route_paths = scenario.case.get("route_region_paths", {})
    for leg, hints in scenario.case.get("route_hints", {}).items():
        missing = [hint for hint in hints if hint not in route_paths.get(leg, [])]
        if missing:
            errors.append(f"route hints not used on {leg}: {missing}")

    return errors


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("cases", nargs="*", help="Case JSON files. Defaults to data/*_case.json.")
    parser.add_argument(
        "--allow-switch-states",
        action="store_true",
        help="Allow fallback s_drop_line/s_home_line states.",
    )
    args = parser.parse_args()

    paths = [Path(item).expanduser().resolve() for item in args.cases]
    if not paths:
        paths = sorted((DEMO_ROOT / "data").glob("*_case.json"))

    ok = True
    for path in paths:
        errors = verify_case(path, strict_concise=not args.allow_switch_states)
        if errors:
            ok = False
            print(f"[FAIL] {path}")
            for error in errors:
                print(f"  - {error}")
            continue
        scenario = PayloadTransportScenario(case_file=str(path))
        print(
            f"[OK] {path.name}: "
            f"regions={len(scenario.regions)} bridges={len(scenario.bridges)} states={len(scenario.states)} "
            f"workspace={scenario.min_workspace_clearance():.1f}/{scenario.workspace_robot_margin:.1f}px"
        )
    if not ok:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
