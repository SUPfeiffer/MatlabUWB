import random

import trajectories

def traj1():
    # square at 2m
    setpoints = []
    setpoints += trajectories.takeoff([0,0], 2.0, 0.0)
    setpoints += trajectories.xySquare(2.0, 2.0, 0.0)
    setpoints += trajectories.land([0,0], 2.0, 0.0)
    return setpoints

def traj2():
    # octagon at 1m
    setpoints = []
    setpoints += trajectories.takeoff([0,0], 1.0, 0.0)
    setpoints += trajectories.xyPolygon(8, 2.0, 1.0, 0.0)
    setpoints += trajectories.land([0,0], 1.0, 0.0)
    return setpoints

def traj3():
    # triangle
    setpoints = []
    setpoints += trajectories.takeoff([0,0], 1.5, 0.0)
    setpoints += trajectories.xyPolygon(3, 2.0, 1.5, 0.0)
    setpoints += trajectories.land([0,0], 1.5, 0.0)
    return setpoints

def traj4():
    # hourglass
    setpoints = []
    setpoints += trajectories.takeoff([0,0], 1.0, 0.0)
    setpoints.append( (2.0, 2.0, 1.0, 0.0) )
    setpoints.append( (2.0, -2.0, 1.0, 0.0) )
    setpoints.append( (-2.0, 2.0, 1.0, 0.0) )
    setpoints.append( (-2.0, -2.0, 1.0, 0.0) )
    setpoints += trajectories.land([0,0], 1.0, 0.0)
    return setpoints

def traj5():
    # random
    setpoints = []
    setpoints += trajectories.takeoff([0,0], 1.5, 0.0)
    points = 10
    for i in range(points):
        x = random.uniform(-2.5, 2.5)
        y = random.uniform(-2.5, 2.5)
        setpoints.append( (x, y, 1.5, 0.0) )
    setpoints += trajectories.land([0,0], 1.5, 0.0)
    return setpoints

def traj6():
    # scan
    setpoints = []
    setpoints += trajectories.takeoff([0,0], 1.0, 0.0)
    setpoints += trajectories.scanArea([-3.0, 3.0, -3.0, 3.0], 0.5, 1.0, 0.0)
    setpoints += trajectories.land([0,0], 1.0, 0.0)
    return setpoints

    