# Standard QP Demo Index

These are the standard demos for this standalone pipeline.

| # | Case file | Source map | Notes |
|---|---|---|---|
| 1 | `data/demo1_case.json` | `data/demo1_map.json` | Managed baseline paper-QP demo 1 |
| 2 | `data/demo2_case.json` | `data/demo2_map.json` | Managed baseline paper-QP demo 2 |
| 3 | `data/demo3_case.json` | `data/demo3_map.json` | Managed baseline paper-QP demo 3 |
| 4 | `data/demo4_case.json` | `data/demo4_map.json` | Managed baseline paper-QP demo 4 |
| 5 | `data/demo5_case.json` | `data/demo5_map.json` | User-provided map 1 |
| 6 | `data/demo6_case.json` | `data/demo6_map.json` | User-provided map 2 |
| 7 | `data/demo7_case.json` | `data/demo7_map.json` | User-provided map 3 |
| 8 | `data/demo8_case.json` | `data/demo8_map.json` | User-provided map 4; generated with automatic sampling |

Run by number:

```bash
./scripts/run_demo.sh 1
./scripts/run_demo.sh 2
./scripts/run_demo.sh 3
./scripts/run_demo.sh 4
./scripts/run_demo.sh 5
./scripts/run_demo.sh 6
./scripts/run_demo.sh 7
./scripts/run_demo.sh 8
```

Verify all:

```bash
./scripts/verify_standard_qp_demos.sh
```

Rebuild all:

```bash
./scripts/rebuild_standard_qp_demos.sh
```
