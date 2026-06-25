# Paper-QP Controller Notes

Main file:

```text
src/swarm_random_payload/swarm_random_payload/paper_qp_controller.py
```

## What The QP Controls

The model uses three robots with single-integrator dynamics:

```text
x_dot_i = u_i
```

Each QP step chooses the robot velocity commands:

```text
u1x, u1y, u2x, u2y, u3x, u3y
```

The implementation also uses slack-like variables:

```text
delta1
delta2
```

## What The Objective Means

The QP objective keeps control effort reasonable while allowing the constraints to drive the actual behavior.

In plain language:

```text
move as gently as possible, but satisfy convergence and safety constraints
```

## CLF Constraints

CLF means Control Lyapunov Function.

This pipeline uses CLF-style constraints for:

- centroid-to-target convergence,
- formation-shape convergence.

Plain language:

```text
the swarm center should move toward the current target,
and the three robots should reshape into the requested formation.
```

## CBF Constraints

CBF means Control Barrier Function.

This pipeline uses CBF-style constraints for:

- robot-robot collision avoidance,
- robot-obstacle collision avoidance,
- workspace safety.

Plain language:

```text
even while moving toward the target, the chosen velocity must not reduce safety below the allowed margin.
```

## What To Look For In Code

In `paper_qp_controller.py`, read these parts first:

- `PaperQPConfig`: QP parameters and tolerances.
- `PaperQPController.step(...)`: one optimization step.
- pairwise safety constraints: robot-to-robot distance.
- obstacle constraints: robot-to-obstacle distance.
- CLF rows: centroid and formation convergence.
- the returned diagnostic fields: solver success, max constraint violation, `delta1`, `delta2`.

## How It Differs From The Full Paper

This implementation follows the paper-QP structure, but it is still an engineering version:

- the upper route is procedural rather than full GR(1)/TuLiP synthesis,
- obstacle CBFs use geometric signed-distance-style constraints against map obstacles,
- QP parameters are tuned for the pixel-map demo scale,
- the ROS visualization is 2D and uses single-integrator execution.
