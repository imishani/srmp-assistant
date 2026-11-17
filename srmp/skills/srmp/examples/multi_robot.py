"""Plan two panda arms at the same time with xECBS (joint-space goals only)."""
import numpy as np

import srmp

planner = srmp.PlannerInterface()
home = np.radians([0, -45, 0, -135, 0, 90, 45])

# Two copies of the same registry robot need distinct names; place their bases 1 m apart.
for name, y in [("panda0", -0.5), ("panda1", 0.5)]:
    planner.add_robot("panda", name=name)
    planner.set_base_pose(name, srmp.Pose(np.array([0.0, y, 0.0]), np.array([1.0, 0.0, 0.0, 0.0])))
    planner.set_qpos(name, home)

names = ["panda0", "panda1"]
goals_q = {"panda0": home + np.radians([30, 10, 0, 10, 0, -10, 0]),
           "panda1": home + np.radians([-30, 10, 0, 10, 0, -10, 0])}

# ECBS / xECBS need two or more articulations; per-robot keys are optional.
planner.make_planner(names, {"planner_id": "xECBS"})
trajs = planner.plan_multi(
    {n: home for n in names},
    {n: srmp.GoalConstraint(srmp.GoalType.JOINTS, [goals_q[n]]) for n in names},
)

# plan_multi returns {name: Trajectory}; it returns an empty dict when no plan was found.
if not trajs:
    raise SystemExit("no plan found")
for n in names:
    print(n, len(trajs[n].positions), "waypoints, times", trajs[n].times)
for n in names:
    assert len(trajs[n].positions) > 0
