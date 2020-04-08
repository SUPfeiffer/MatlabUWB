'''
Provides functions that return trajectories in form of a list of setpoints
Author: Sven Pfeiffer, MAVLab, TU Delft
'''
import numpy as np

def takeoff(position, altitude, yaw):
    x = position[0]
    y = position[1]
    setpoints = [(x, y, altitude, yaw)]
    return setpoints

def land(position, altitude, yaw):
    x = position[0]
    y = position[1]
    setpoints = [
        (x, y, altitude, yaw),
        (x, y, 0, yaw)
        ]
    return setpoints

# starts at x_min, y_min
def scanArea(limits, spacing, altitude, yaw):
    x_min = limits[0]
    x_max = limits[1]
    y_min = limits[2]
    y_max = limits[3]

    setpoints = []
    x_range = x_max - x_min
    n_lines = int(np.floor(x_range/spacing) + 1)
    y_is_min = True
    for i in range(n_lines):
        x_line = x_min + i * spacing
        y = y_min if y_is_min else y_max
        setpoints.append( (x_line, y, altitude, yaw) )
        y_is_min = not(y_is_min)
        y = y_min if y_is_min else y_max
        setpoints.append( (x_line, y, altitude, yaw) )
    return setpoints

# starts at 0,0
def xySquare(sideLength, altitude, yaw ):
    x_min = -sideLength/2
    x_max = sideLength/2
    y_min = -sideLength/2
    y_max = sideLength/2

    setpoints = [ 
        (x_min, y_min, altitude, yaw),
        (x_min, y_max, altitude, yaw),
        (x_max, y_max, altitude, yaw),
        (x_max, y_min, altitude, yaw),
        (x_min, y_min, altitude, yaw),
        ]
    return setpoints

def xyPolygon(N, radius, altitude, yaw):
    setpoints = []
    for i in range(N):
        angle = 2*np.pi*i/N
        x = radius * np.cos(angle)
        y = radius * np.sin(angle)
        setpoints.append( (x, y, altitude, yaw) )
    setpoints.append( setpoints[0] )
    return setpoints

def multiLevelSquare(sideLength, altitude, yaw, qty):
    x_min = -sideLength/2
    x_max = sideLength/2
    y_min = -sideLength/2
    y_max = sideLength/2

    setpoints = [ (0, 0, altitude, yaw) ]

    for i in range(qty):
        setpoints.append((x_min, y_min, altitude, yaw))
        setpoints.append((x_min, y_max, altitude, yaw))
        setpoints.append((x_max, y_max, altitude, yaw))
        setpoints.append((x_max, y_min, altitude, yaw))
        setpoints.append((x_min, y_min, altitude, yaw))
    
    setpoints.append((0, 0, altitude, yaw))
    setpoints.append((0, 0, 0, yaw))
    return setpoints



# for debugging
if __name__ == "__main__":
    setpoints = xyPolygon(4, 1, 1, 0)
    for i in range(len(setpoints)):
        print(setpoints[i])