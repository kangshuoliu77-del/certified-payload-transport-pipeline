#!/usr/bin/env python3
"""Build a reusable ROS visualization case from one exported map JSON."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import shutil
from types import SimpleNamespace

from generate_case import generate


DEMO_ROOT = Path(__file__).resolve().parents[1]
DATA_DIR = DEMO_ROOT / "data"
PACKAGE_DATA_DIR = DEMO_ROOT / "src" / "swarm_random_payload" / "data"
DEFAULT_PAPER_QP_CONFIG = {
    "timeStep": 0.02,
    "fixedTimeBound": 12.0,
    "uMaxMetersPerSecond": 10.0,
    "delta1LinearWeight": 0.0,
    "constraintTolerance": 0.001,
    "targetTolerance": 1.3,
    "enforceDiscreteSafety": True,
}


def case_slug(raw: str) -> str:
    slug = re.sub(r"[^A-Za-z0-9_]+", "_", raw.strip()).strip("_").lower()
    if slug.endswith("_map"):
        slug = slug[:-4]
    if slug.endswith("_case"):
        slug = slug[:-5]
    return slug or "payload_case"


def write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def copy_if_different(source: Path, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    if source.resolve() != destination.resolve():
        shutil.copyfile(source, destination)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, help="Map JSON exported from iris_map_designer.html.")
    parser.add_argument("--case-name", default=None, help="Output prefix. Defaults to the input file stem.")
    parser.add_argument("--random-seed", type=int, default=31)
    parser.add_argument("--seed-budget", type=int, default=96)
    parser.add_argument("--max-regions", type=int, default=36)
    parser.add_argument("--min-regions", type=int, default=10)
    parser.add_argument("--iteration-limit", type=int, default=60)
    parser.add_argument("--min-region-area", type=float, default=900.0)
    parser.add_argument("--obstacle-overlap-tolerance", type=float, default=1.0)
    parser.add_argument("--obstacle-margin", type=float, default=None)
    parser.add_argument("--paper-qp", action="store_true", help="Emit a paper_qp-controlled case.")
    parser.add_argument(
        "--source-regions-only",
        action="store_true",
        help="Use only seeds from exported source regions; useful for preserving a compact accepted region set.",
    )
    args = parser.parse_args()

    source_path = Path(args.input).expanduser().resolve()
    name = case_slug(args.case_name or source_path.stem)
    map_out = DATA_DIR / f"{name}_map.json"
    package_map_out = PACKAGE_DATA_DIR / f"{name}_map.json"
    case_out = DATA_DIR / f"{name}_case.json"
    package_case_out = PACKAGE_DATA_DIR / f"{name}_case.json"

    DATA_DIR.mkdir(parents=True, exist_ok=True)
    PACKAGE_DATA_DIR.mkdir(parents=True, exist_ok=True)
    copy_if_different(source_path, map_out)
    copy_if_different(source_path, package_map_out)
    if args.source_regions_only or args.obstacle_margin is not None:
        for path in (map_out, package_map_out):
            payload = json.loads(path.read_text(encoding="utf-8"))
            if args.source_regions_only:
                payload["sourceRegionsOnly"] = True
            if args.obstacle_margin is not None:
                payload["obstacleMargin"] = args.obstacle_margin
            write_json(path, payload)

    gen_args = SimpleNamespace(
        input=str(map_out),
        output=str(case_out),
        package_output=str(package_case_out),
        random_seed=args.random_seed,
        seed_budget=args.seed_budget,
        max_regions=args.max_regions,
        min_regions=args.min_regions,
        iteration_limit=args.iteration_limit,
        min_region_area=args.min_region_area,
        obstacle_overlap_tolerance=args.obstacle_overlap_tolerance,
        source_regions_only=args.source_regions_only,
    )
    case = generate(gen_args)
    if args.paper_qp:
        case["schema"] = "swarm_random_payload_case.paper_qp.v1"
        case["control"] = {
            "mode": "paper_qp",
            "paper_qp": dict(DEFAULT_PAPER_QP_CONFIG),
        }
    write_json(case_out, case)
    write_json(package_case_out, case)

    print(
        f"built case '{name}': {len(case['regions'])} regions, "
        f"{len(case['bridges'])} selected bridges, {len(case['states'])} states"
    )
    print(f"map:  {map_out}")
    print(f"case: {case_out}")
    print()
    print("Run after building the ROS package:")
    print(f"cd {DEMO_ROOT}")
    print("source /opt/ros/humble/setup.bash")
    print("colcon build --packages-select swarm_random_payload")
    print("source install/setup.bash")
    print(f"ros2 launch swarm_random_payload case_payload_demo.launch.py case_file:={case_out.resolve()}")


if __name__ == "__main__":
    main()
