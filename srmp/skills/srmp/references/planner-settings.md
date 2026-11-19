# Planner settings (`make_planner`)

```python
# skill-test: skip (signature only)
planner.make_planner(articulation_names: list[str], planner_context: dict[str, str]) -> None
```

- Every value is a **string**: `{"weight": "50"}`, not `{"weight": 50}`. A number raises
  `TypeError`. The only list-valued key is `inadmissible_heuristics` (MHAstar).
- Keys and values are case-sensitive. Unknown keys are ignored silently, so check spelling.
  Unknown `planner_id`, heuristic or `validity_backend` values raise `RuntimeError`.
- Call `make_planner` again whenever you change robots, planners or settings. Planning many
  queries with the same planner is cheap: built state is reused while the scene, goal and
  settings stay the same.
- Defaults below are what srmp 0.2.1–0.3.0 apply when a key is omitted (from the source; the
  logged ones were checked against the build). Planners log many of the defaults they fill
  in, for example `No weight specified. Using default: 50`.

## `planner_id` values

| planner_id | Robots | Goals | Notes |
|---|---|---|---|
| `wAstar` | one | joint, pose | Weighted A*. Start here. |
| `Astar` | one | joint, pose | Plain A* (weight 1); slow on large problems. |
| `ARAstar` | one | joint, pose | Anytime: finds a path at `weight`, then improves it until `final_weight` or the time limit. |
| `MHAstar` | one | joint, pose | Multi-heuristic A*: an anchor heuristic plus inadmissible ones. |
| `wPASE` | one | joint, pose | Weighted PASE: expands states in parallel threads. |
| `MGS` | one | joint, pose | Multi-graph search: grows several graphs and connects them; for hard scenes. |
| `vamp_rrtc` | one | joint only | VAMP RRT-Connect (sampling). Needs `validity_backend` `vamp` or `auto`, else `plan()` returns an empty trajectory. |
| `ECBS` | two or more | joint only | Multi-robot conflict-based search. |
| `xECBS` | two or more | joint only | Experience-accelerated ECBS. |

Single-robot planners take one articulation name; `ECBS` / `xECBS` take two or more (with one
name they raise `Unknown planner`). `planner.print_available_planners()` lists what the
installed build supports.

## `heuristic` values

| heuristic | Meaning |
|---|---|
| `bfs` | Default for all planners except MHAstar, wPASE and MGS. BFS over the occupancy grid from the end-effector goal: routes around obstacles; best in clutter. Not allowed as MGS `heuristic` (use it as `anchor_heuristic`). |
| `joint_euclidean` | Euclidean distance in joint space; cheap, ignores obstacles. |
| `joint_euclidean_remove_time` | Joint-space Euclidean distance ignoring a time dimension (for timed states). |
| `joint_steps` | Joint-space distance counted in discretization steps. MGS default. |
| `joint_time` | Straight-line move duration at the joint velocity limits. |
| `joint_time_floor` | `joint_time` plus an obstacle-aware floor term for mobile bases; equals `joint_time` on a fixed arm. |

For `ECBS` / `xECBS` only `bfs`, `joint_euclidean` and `joint_euclidean_remove_time` are
valid, set per robot with `heuristic_<robot name>`.

## `validity_backend` values

| validity_backend | Meaning |
|---|---|
| `coal` | Default. Mesh collision checking with COAL; works for every robot. |
| `vamp` | SIMD sphere-based checking, much faster. Raises if the robot has no matching VAMP model. Fingers are fixed in the VAMP model. |
| `auto` | `vamp` when every planned robot supports it, else `coal`. Never raises. |

Check with `planner.get_available_validity_backends()` (backends in this build),
`planner.is_robot_vamp_supported(name)` and, after `make_planner`,
`planner.get_active_validity_backend()`. See [vamp.md](vamp.md).

## Keys used by every single-robot planner

| Key | Default | Meaning |
|---|---|---|
| `planner_id` | (required) | One of the planner ids above. |
| `time_limit` | `"10"` | Seconds before giving up. `allowed_planning_time` is an alias. Not read by `ECBS` / `xECBS`. |
| `heuristic` | per planner | See the heuristic table. |
| `resolution` | `"1"` | Joint discretization in degrees (revolute joints). |
| `resolution_prismatic` | `"0.05"` | Discretization of prismatic joints, metres. |
| `mprim_path` | built-in | Motion-primitive YAML file; default is the built-in set for the robot's joint count. |
| `validity_backend` | `"coal"` | See above. |
| `verbose` | `"false"` | `"true"` or `"1"` prints search progress. |
| `ik_max_iter` | `"50"` | IK iterations per solve (pose goals). |
| `ik_eps` | `"1e-5"` | IK convergence tolerance. |
| `snap_cached_goals` | `"3"` | Goal configurations kept and reused as snap targets. |

## Per-planner keys

| Planner | Key | Default | Meaning |
|---|---|---|---|
| `wAstar` | `weight` | `"50"` | Heuristic inflation. Higher: faster, longer paths. |
| `ARAstar` | `weight` | `"50"` | Starting weight. |
| `ARAstar` | `weight_delta` | `"10.0"` | Weight decrease per improvement round. |
| `ARAstar` | `final_weight` | `"1.0"` | Stop improving at this weight. |
| `MHAstar` | `heuristic` | `"joint_euclidean"` | Anchor (admissible) heuristic. |
| `MHAstar` | `inadmissible_heuristics` | `["bfs"]` | List of heuristic names (a single string also works). |
| `MHAstar` | `w1` | `"20"` | Inflation of each heuristic. |
| `MHAstar` | `w2` | `"5"` | Bound of inadmissible queues relative to the anchor. |
| `wPASE` | `heuristic` | `"joint_euclidean"` | Main heuristic. |
| `wPASE` | `i_heuristic` | `"joint_euclidean"` | Independence-check heuristic. |
| `wPASE` | `weight` | `"50"` | Heuristic inflation. |
| `wPASE` | `i_weight` | `"100.0"` | Weight for `i_heuristic`. |
| `wPASE` | `num_threads` | `"4"` | Worker threads. |
| `MGS` | `heuristic` | `"joint_steps"` | Root-to-root distance. `bfs` raises. |
| `MGS` | `anchor_heuristic` | `"bfs"` | Distance-to-goal for the anchor search. |
| `MGS` | `weight` | `"100"` | Suboptimality bound of the anchor (start) graph search. |
| `MGS` | `g_num` | `"10"` | Max graphs including start and goal; at least `"2"` (else raises). |
| `MGS` | `graph_growth` | `"on_stall"` | `"on_stall"` or `"upfront"`. |
| `MGS` | `stall_window` | `"50"` | Expansions without a better heuristic value before a root graph is added. |
| `MGS` | `root_interval` | `"0"` | Add a root every N expansions; `"0"` is off. |
| `MGS` | `root_selection` | `"frontier_nearest"` | `"frontier_nearest"` or `"sequential"`. |
| `MGS` | `goal_roots` | `"1"` | Pose goals: `"1"` roots a goal graph at an IK solution of the goal pose (bidirectional search); `"0"` searches from the start graph only. |
| `MGS` | `root_clearance` | `"0.05"` | Minimum end-effector clearance of a root, metres. |
| `MGS` | `nonblocking_roots` | `"false"` | `"true"`: keep expanding while roots are prepared in the background (non-deterministic timing). |
| `MGS` | `anchor_search` | `"true"` | After the first start-goal connection, keep searching to bound the cost by `weight`. |
| `MGS` | `anchor_orientation_weight` | `"0"` | Cost per radian of end-effector orientation error in the anchor heuristic. |
| `MGS` | `lazy_edges` | `"false"` | Lazy edge evaluation: motions are collision-checked only when a state is expanded. |
| `MGS` | `root_prefetch`, `root_ik_threads`, `expansion_threads`, `snap_ik_seeds` | build-chosen | Thread and prefetch counts; leave unset unless tuning. |
| `vamp_rrtc` | `rrtc_range`, `rrtc_max_iterations`, `rrtc_max_samples`, `rrtc_simplify` | VAMP's defaults | RRT-Connect step size, iteration and sample caps, path simplification (`"true"`/`"false"`). |

## Multi-robot keys (`ECBS`, `xECBS`)

| Key | Default | Meaning |
|---|---|---|
| `heuristic_<robot name>` | `"bfs"` | Per-robot heuristic (time-aware versions are used automatically). |
| `mprim_path_<robot name>` | built-in timed primitives | Per-robot primitives file. |
| `resolution_<robot name>` | `"1"` | Per-robot discretization, degrees. |
| `weight_low_level_heuristic` | see note | Inflation of the per-robot searches. |
| `high_level_focal_suboptimality` | `"1.3"` | Suboptimality bound of the conflict tree search. |
| `low_level_focal_suboptimality` | `"1.3"` | Suboptimality bound of the per-robot searches. |

Note on `weight_low_level_heuristic`: when it is omitted, srmp 0.2.1–0.3.0 logs "Using default:
1.0" but the source applies `50.0`. Set it explicitly (for example `"1.0"` or `"50"`) when the
value matters. See [multi-robot.md](multi-robot.md).

## VAMP scene translation keys

Used when the VAMP backend is active. All metres except `attachment_max_spheres`.

| Key | Default | Meaning |
|---|---|---|
| `mesh_sampling_resolution` | `"0.01"` | Surface sampling spacing for mesh obstacles and attached meshes. |
| `pointcloud_point_radius` | `"0.01"` | Radius given to every sampled or point-cloud point. |
| `articulation_link_sampling_resolution` | `"0.05"` | Sampling spacing for mesh links of robots that are not planned. |
| `attachment_spherization_resolution` | `"0.05"` | Sphere pitch for attached primitives (boxes, cylinders). |
| `attachment_max_spheres` | `"512"` | Sphere cap per attached object. Primitives are coarsened to fit; an attached mesh over the cap raises: raise it, or attach a primitive such as a box. |

## Example

```python
planner.add_robot("panda")
start = np.radians([0, -45, 0, -135, 0, 90, 45])
goal = srmp.GoalConstraint(srmp.GoalType.JOINTS, [start + np.radians([40, 0, 0, 0, 0, 0, 0])])

for ctx in [
    {"planner_id": "wAstar", "heuristic": "bfs", "weight": "50", "time_limit": "5"},
    {"planner_id": "ARAstar", "weight": "50", "final_weight": "1.0", "time_limit": "2"},
    {"planner_id": "MHAstar", "heuristic": "joint_euclidean", "inadmissible_heuristics": ["bfs"]},
    {"planner_id": "MGS", "g_num": "6", "time_limit": "5"},
    {"planner_id": "wAstar", "validity_backend": "auto"},
]:
    planner.make_planner(["panda"], ctx)
    traj = planner.plan(start, goal)
    assert len(traj.positions) > 0, ctx
    print(ctx["planner_id"], planner.get_active_validity_backend(), len(traj.positions))
```
