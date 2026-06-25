# Pipeline Overview

The pipeline turns a map into an executable ROS visualization case.

## 1. Map Input

Input file:

```text
data/demo*_map.json
```

A map contains:

- obstacles,
- HOME / PICK / DROP / RETURN task points,
- formation scale,
- safe distance,
- safety margin,
- optional manual seed points.

## 2. IRIS Region Generation

Implemented in:

```text
tools/generate_case.py
figures/drake_iris_server.py
```

The generator samples points in free space and calls Drake IRIS at each useful seed. Each accepted IRIS region is a convex safe region that does not intersect inflated obstacles.

## 3. Formation Feasibility

Implemented in:

```text
tools/generate_case.py
```

For each region, the generator checks whether the line or triangle formation envelope can fit inside the region while respecting the safe distance.

The result is:

```text
Allowed(P_i) = {line, triangle}
```

## 4. Bridge Construction

Implemented in:

```text
tools/generate_case.py
```

For nearby/intersecting IRIS regions, the generator tries to place a formation inside the region intersection. If successful, that intersection becomes a bridge state.

This is what lets the symbolic route move from one region to another without ignoring formation size.

## 5. Symbolic Route

Implemented in:

```text
tools/generate_case.py
```

Current fixed task order:

```text
HOME -> PICK -> DROP -> RETURN -> HOME
```

Current formation logic:

- starts at HOME in triangle,
- travels HOME -> PICK in line,
- switches to triangle for loaded PICK -> DROP,
- switches back to line for DROP -> RETURN -> HOME,
- ends at HOME in triangle.

Important: this is not yet full GR(1)/TuLiP synthesis. It is a procedural symbolic route over the generated formation-aware region graph.

## 6. Case JSON

Output file:

```text
data/demo*_case.json
```

A case contains:

- regions,
- bridges,
- states,
- transitions,
- route paths,
- task metadata,
- QP control config,
- verification metadata.

## 7. Paper-QP Execution

Implemented in:

```text
src/swarm_random_payload/swarm_random_payload/random_payload_model.py
src/swarm_random_payload/swarm_random_payload/paper_qp_controller.py
```

The scenario model reads the symbolic states and asks the QP controller to move the robots from one target placement to the next.

The QP controller enforces:

- centroid convergence,
- formation convergence,
- inter-robot safety,
- obstacle safety,
- input bounds,
- workspace bounds.

## 8. ROS Visualization

Implemented in:

```text
src/swarm_random_payload/swarm_random_payload/random_payload_node.py
src/swarm_random_payload/launch/case_payload_demo.launch.py
```

The ROS node publishes the map, regions, bridges, robot markers, payload marker, and route visualization.
