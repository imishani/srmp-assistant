# Collision backends and VAMP

SRMP checks collisions with one of two backends, chosen per planner with the
`validity_backend` key of `make_planner`:

| validity_backend | Behaviour |
|---|---|
| `coal` | Default. Mesh collision checking with COAL. Works for every robot and object. |
| `vamp` | VAMP: SIMD, sphere-based checking; often much faster. Raises in `make_planner` if a planned robot has no matching VAMP model. |
| `auto` | `vamp` when every planned robot supports it, else `coal`. Never raises. |

```python
print(planner.get_available_validity_backends())   # backends compiled into this build, e.g. ['coal', 'vamp']
planner.add_robot("panda")
print(planner.is_robot_vamp_supported("panda"))     # True when a matching VAMP model exists
planner.make_planner(["panda"], {"planner_id": "wAstar", "validity_backend": "auto"})
print(planner.get_active_validity_backend())        # the backend actually in use
```

## When not to use VAMP

- **Grasping.** VAMP's robot model has the gripper fingers fixed at one opening, so finger
  positions you set are not what VAMP checks. When fingers close on or near an object, plan
  with `coal`.
- The robot has no VAMP model. `is_robot_vamp_supported(name)` is the test. It is also
  False when a VAMP module of the same name exists but describes different kinematics
  (VAMP's built-in `ur5` and `fetch` are not the registry robots), so those plan on `coal`.
- `get_vamp_supported_robots()` lists the VAMP module names in this build (including
  installed plugins); a name in that list does not guarantee `is_robot_vamp_supported`.

## Checking a single configuration

- `planner.is_state_colliding(name)` (coal): **True means in collision**.
- `planner.is_vamp_state_valid(name, q)` (VAMP): **True means collision-free**. Opposite
  polarity. Only for robots where `is_robot_vamp_supported(name)` is True.

## A base resting on a table

With the base flush on a table, every state collides, under both backends.
`planner.set_base_contact_allowed(name, True)` stops checking the robot's fixed links
(`get_fixed_link_names`) against static objects, but only under coal: VAMP's base checks are
compiled in. With the switch on and a base sphere overlapping an object, `"auto"` falls back
to coal and `"vamp"` raises, naming the object. Lowering or cutting out the table under the
base keeps VAMP.

## The VAMP-only planner

`vamp_rrtc` is VAMP's RRT-Connect. It needs the VAMP backend (`"validity_backend": "vamp"`
or `"auto"`) and a joint goal. Without VAMP, or with a pose goal, `plan()` logs the reason
and returns an empty trajectory.

```python
# skill-test: needs-vamp
planner.add_robot("panda")
start = np.radians([0, -45, 0, -135, 0, 90, 45])
goal = srmp.GoalConstraint(srmp.GoalType.JOINTS, [start + np.radians([60, 0, 0, -10, 0, 0, 0])])
planner.make_planner(["panda"], {"planner_id": "vamp_rrtc", "validity_backend": "vamp",
                                 "time_limit": "5"})
traj = planner.plan(start, goal)
assert len(traj.positions) > 0
```

## Scene translation

With VAMP, mesh obstacles, point clouds, unplanned robots and attached objects are converted
to points or spheres. Tune with these keys (metres, as strings; details in
[planner-settings.md](planner-settings.md)): `mesh_sampling_resolution` (`"0.01"`), `pointcloud_point_radius` (`"0.01"`),
`articulation_link_sampling_resolution` (`"0.05"`), `attachment_spherization_resolution`
(`"0.05"`), `attachment_max_spheres` (`"512"`; an attached mesh over the cap raises: raise
it, or attach a primitive such as a box).

## VAMP plugins for more robots

VAMP models for more robots come as plugins, without recompiling srmp.

- `planner.add_robot("<registry name>")` installs and loads the published prebuilt plugin
  automatically when one exists for this srmp build. The first time this downloads the
  plugin and runs a short agreement test, which can take a few minutes.
- `srmp.vamp_plugins.plugin_dirs()` lists plugin directories (`$SRMP_VAMP_PLUGIN_PATH`, then
  `~/.cache/srmp/vamp_plugins`); `srmp.vamp_plugins.load_plugins()` reloads installed ones.

Managing plugins is a shell command, not Python:

```text
srmp vamp list                        # installed plugins, load status, agreement-gate state
srmp vamp fetch <registry name>       # download the published plugin for a registry robot
srmp vamp remove <name>               # delete an installed plugin
srmp vamp rebuild-all                 # rebuild plugins an srmp upgrade left on an old ABI
srmp vamp add-robot --name <name> ...   # build a plugin for a new robot (see --help)
```

`srmp vamp add-robot` builds from a sphere-only (spherized) URDF (`--urdf`, with
`--end-effector`) or a cricket-generated module (`--from-module`), runs the build in a
container when Docker is usable, and requires exactly one of `--spherized-urdf` (agreement
test) or `--skip-verify`. Run `srmp vamp add-robot --help` for the full options.
