# Choosing a planner

All search-based planners in SRMP search a lattice of motion primitives. They are
deterministic: the same scene and query give the same path. Settings and defaults are in
[planner-settings.md](planner-settings.md).

## Start with `wAstar`

```python
planner.add_robot("panda")
planner.make_planner(["panda"], {"planner_id": "wAstar", "heuristic": "bfs",
                                 "weight": "50", "time_limit": "10"})
```

- `weight` (default `"50"`): higher plans faster with longer paths; lower gives shorter
  paths but takes longer. Useful range is roughly `"10"` to `"100"`.
- `heuristic`: `bfs` (default) routes the end effector around obstacles and suits cluttered
  scenes. `joint_euclidean` is cheaper but ignores obstacles.
- `time_limit` (default `"10"` seconds): on timeout `plan()` returns an empty trajectory.

## When to switch

| Situation | Use |
|---|---|
| Default, most single-arm queries | `wAstar` |
| Small problem, want the cheapest lattice path | `Astar` (often much slower) |
| Fixed time budget, want the path to improve with time | `ARAstar` (`weight`, `weight_delta`, `final_weight`, `time_limit`) |
| One heuristic gets stuck; want several with a cost bound | `MHAstar` (`heuristic` anchor + `inadmissible_heuristics`) |
| Collision checks are expensive and cores are free | `wPASE` (`num_threads`) |
| Hard scenes where `wAstar` stalls: shelves, narrow openings, deep pose goals | `MGS` (more setup per query) |
| Raw speed on a VAMP-supported robot, joint goal, no lattice guarantees | `vamp_rrtc` with `validity_backend` `vamp` or `auto` |
| Several robots moving at the same time | `xECBS` or `ECBS`, see [multi-robot.md](multi-robot.md) |

`MGS` rejects `bfs` as its `heuristic`; its default is `joint_steps`, and `bfs` is its
`anchor_heuristic`. `vamp_rrtc` does not accept pose goals: solve IK first
(`collision_aware_ik`) and plan to the joint configuration.

## Making planning faster

- Use the VAMP backend (`"validity_backend": "auto"`): sphere-based SIMD collision checks,
  often far faster than mesh checks. Keep `coal` for grasping motions, since VAMP fixes the
  finger opening. See [vamp.md](vamp.md).
- Raise `weight` if path length matters less than planning time.
- Size the occupancy grid to the scene (`srmp.GridConfig`, passed to
  `srmp.PlannerInterface(grid_config)` or `planner.set_grid(...)`). An oversized or very fine
  grid slows the `bfs` heuristic. Defaults: origin `(-1, -1, 0)`, size `2 x 2 x 2` m,
  resolution `0.02` m.
- Create the planner once and plan many times; built state is reused while the scene and
  goal stay the same.
- After a plan, `planner.get_planning_stats()` shows where the time went (`time`, `cost`,
  `num_expanded`, and a `bonus_stats` dict).

## Pose goals vs joint goals

- Joint goal: `srmp.GoalConstraint(srmp.GoalType.JOINTS, [q])`, radians, move-group order.
- Pose goal: `srmp.GoalConstraint(srmp.GoalType.POSE, [pose])`, the end-effector pose in the
  world frame, `pose.q` as `[w, x, y, z]`. The search planners reach it by snapping to IK
  solutions near the goal.
- `GoalType` also lists `POSITION`, `MULTI_GOAL_JOINTS`, `MULTI_GOAL_POSE` and
  `MULTI_GOAL_POSITION`, but in srmp 0.2.1–0.3.0 `plan()` raises "Goal type not supported at the
  moment" for them, and the list constructor accepts exactly one goal. Use `JOINTS` or `POSE`
  with a one-element list. For alternatives, plan to each candidate and keep the best.

```python
planner.add_robot("panda")
start = np.radians([0, -45, 0, -135, 0, 90, 45])
ee = planner.compute_fk("panda", start)          # never guess positions
target = srmp.Pose(ee.p + np.array([0.15, 0.10, -0.05]), ee.q.copy())
planner.make_planner(["panda"], {"planner_id": "wAstar", "weight": "50"})
traj = planner.plan(start, srmp.GoalConstraint(srmp.GoalType.POSE, [target]))
assert len(traj.positions) > 0
print(planner.get_planning_stats()["time"], "s planning,", len(traj.positions), "waypoints")
```
