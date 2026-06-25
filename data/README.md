# Data

Canonical demo data for the pipeline.

`demoN_map.json`
: Human/map-designer input. Contains obstacles, task regions, return point,
  formation scale, safety distances, and seed settings.

`demoN_case.json`
: Generated executable case. Contains certified regions, formation-feasible
  states, bridges, route states, transitions, task metadata, and QP settings.

`case_manifest.json`
: Manifest used by `tools/rebuild_cases.py` and
  `scripts/rebuild_standard_qp_demos.sh`.

The same case/map files are copied into:

```text
src/swarm_random_payload/data/
```

so ROS can install and locate them through the package share directory.
