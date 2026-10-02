from go2_sim.world import KinematicRobot, World, footprint_clearance  # noqa: F401


def run_planner(world, planner, start, goal, dt: float = 0.05, t_max: float = 60.0,
                goal_tol: float = 0.25, side_hint: int = 0):
    robot = KinematicRobot(start)
    log = []
    t = 0.0
    while t < t_max:
        scan = world.raycast(robot.pose)
        out = planner.compute(t, robot.pose, scan, goal, side_hint)
        log.append((t, robot.pose, out))
        if out.goal_dist < goal_tol:
            break
        robot.step(out.vx, out.wz, dt)
        t += dt
    return robot, log


def run_mission(world, mission, start, dt: float = 0.05, t_max: float = 200.0):
    robot = KinematicRobot(start)
    mission.start()
    log = []
    t = 0.0
    while t < t_max:
        out = mission.step(t, robot.pose, world.raycast(robot.pose))
        log.append((t, robot.pose, out))
        if mission.done:
            break
        robot.step(out.vx, out.wz, dt)
        t += dt
    return robot, log
