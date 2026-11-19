# Multi-robot planning (`ECBS`, `xECBS`)

`ECBS` (Enhanced Conflict-Based Search) and `xECBS` (Experience-Accelerated ECBS) plan
two or more robots together so their motions do not collide with each other. Both search
timed states (joint values plus a time step) and accept **joint-space goals only**.

## Workflow

```python
home = np.radians([0, -45, 0, -135, 0, 90, 45])
names = ["panda0", "panda1"]

# Distinct names for each copy; place the bases apart. Identity orientation is [1, 0, 0, 0].
for name, y in zip(names, [-0.5, 0.5]):
    planner.add_robot("panda", name=name)
    planner.set_base_pose(name, srmp.Pose(np.array([0.0, y, 0.0]), np.array([1.0, 0.0, 0.0, 0.0])))
    planner.set_qpos(name, home)

planner.make_planner(names, {"planner_id": "xECBS"})
starts = {n: home for n in names}
goals = {n: srmp.GoalConstraint(srmp.GoalType.JOINTS, [home + np.radians([30, 10, 0, 10, 0, -10, 0])])
         for n in names}
trajs = planner.plan_multi(starts, goals)

if not trajs:                     # empty dict: no plan found
    raise RuntimeError("no multi-robot plan found")
for n in names:
    assert len(trajs[n].positions) > 0
    print(n, len(trajs[n].positions), "waypoints")
```

## Facts

- `make_planner(names, ...)` with `ECBS` / `xECBS` needs two or more articulation names;
  with one name it raises `Unknown planner`. Single-robot planners plan only one robot.
- `plan_multi(start_states, goal_constraints)` takes dicts keyed by articulation name and
  returns `{name: Trajectory}`. When no plan is found it returns an **empty dict**, so check
  `if not trajs` before indexing.
- A `POSE` goal raises `RuntimeError: Only joint-space goals are supported for multi-agent
  planning`. Convert poses to joint goals first with `collision_aware_ik` (returns
  `(q, status)`) and check the status is `"found"`.
- Each trajectory's `positions` are move-group joint values (no time column). Its `times`
  are the search's time steps (for example `[0.0, 1.0]`), not seconds.
- `time_limit` is **not** read by `ECBS` / `xECBS`. In srmp 0.2.1–0.3.0 a query with no solution
  ran about 60 s before returning an empty dict, whatever `time_limit` said. Keep queries
  feasible (goals reachable, robots not blocking each other's goals) and run long queries in
  a subprocess or worker with a timeout if you must bound wall time.
- Robots not named in `make_planner` stay in the scene as static obstacles (bystanders).
  `add_robot(..., planned=False)` also adds a robot that is never planned.
- `is_state_colliding(name)` did not report contact with another robot in srmp 0.2.1–0.3.0;
  `is_state_colliding()` with no argument checks the whole scene, robots included. A start
  state that touches another robot makes `plan()` log "Start state is not valid" and return
  an empty trajectory.
- `validity_backend` works as for one robot: `"auto"` uses VAMP when every planned robot
  supports it.

## Keys

| Key | Default | Meaning |
|---|---|---|
| `heuristic_<robot name>` | `"bfs"` | `bfs`, `joint_euclidean` or `joint_euclidean_remove_time`. Time-aware versions are used automatically. |
| `mprim_path_<robot name>` | built-in timed primitives | Per-robot primitives file. |
| `resolution_<robot name>` | `"1"` | Per-robot joint discretization, degrees. |
| `weight_low_level_heuristic` | see below | Inflation of each robot's low-level search. |
| `high_level_focal_suboptimality` | `"1.3"` | Focal bound of the high-level (conflict tree) search. |
| `low_level_focal_suboptimality` | `"1.3"` | Focal bound of each robot's low-level search. |
| `validity_backend` | `"coal"` | `coal`, `vamp` or `auto`. |
| `verbose` | `"false"` | `"true"` prints search progress. |

When `weight_low_level_heuristic` is omitted, srmp 0.2.1–0.3.0 logs "Using default: 1.0" but the
source applies `50.0`. Set it explicitly when it matters.

```python
planner.add_robot("panda", name="left")
planner.add_robot("panda", name="right")
planner.set_base_pose("right", srmp.Pose(np.array([0.0, 1.0, 0.0]), np.array([1.0, 0.0, 0.0, 0.0])))
planner.make_planner(["left", "right"], {
    "planner_id": "ECBS",
    "heuristic_left": "bfs",
    "heuristic_right": "joint_euclidean",
    "weight_low_level_heuristic": "1.0",
    "high_level_focal_suboptimality": "1.3",
    "low_level_focal_suboptimality": "1.3",
})
home = np.radians([0, -45, 0, -135, 0, 90, 45])
trajs = planner.plan_multi(
    {"left": home, "right": home},
    {"left": srmp.GoalConstraint(srmp.GoalType.JOINTS, [home + np.radians([20, 0, 0, 0, 0, 0, 0])]),
     "right": srmp.GoalConstraint(srmp.GoalType.JOINTS, [home + np.radians([-20, 0, 0, 0, 0, 0, 0])])},
)
assert trajs and all(len(t.positions) > 0 for t in trajs.values())
```
