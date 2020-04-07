# customUtils.py
# 
# Contains custom utility functions used in the python scripts to control the crazyflie
#
# Author: S.Pfeiffer, MAVLab

import math
import numpy.linalg as npl

RAD2DEG = 180/math.pi
# convert optitrack quaternions into crazyflie euler angles (degrees)
def quat2euler(q):
    q = q/npl.norm(q)
    pitch = RAD2DEG*math.atan2(-2*(q[1]*q[2]-q[0]*q[3]), q[0]**2-q[1]**2+q[2]**2-q[3]**2)
    roll = RAD2DEG*math.asin(2*(q[2]*q[3]+q[0]*q[1]))
    yaw = RAD2DEG*(-math.atan2(-2*(q[1]*q[3]-q[0]*q[2]), q[0]**2-q[1]**2-q[2]**2+q[3]**2))

    if pitch > 0:
        pitch = pitch - 180
    else:
        pitch = pitch + 180

    eulerAngles = [roll, pitch, yaw]

    return eulerAngles


if __name__ == '__main__':
    import numpy as np
    import csv
    import matplotlib.pyplot as plt

    quat = []
    att = []
    t = []
    with open('square.csv') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        line_count = 0
        for row in csv_reader:
            if line_count == 0:
                line_count += 1
            else:
                t.append(float(row[0]))
                quat.append([float(row[4]), float(row[5]), float(row[6]), float(row[7])])
                att.append([float(row[12]), float(row[13]), float(row[14])])
                line_count += 1

        print(f'Processed {line_count} lines.')

    att_c = []
    for q in quat:
        euler = quat2euler(q)
        att_c.append(euler)

    t = np.array(t)
    att = np.array(att)
    att_c = np.array(att_c)

    err_roll = att[:, 0]-att_c[:, 0]
    err_pitch = att[:, 1]-att_c[:, 1]
    err_yaw = att[:, 2]-att_c[:, 2]

    fig, ax = plt.subplots()  # Create a figure and an axes.
    ax.plot(t, err_roll, label='roll')  # Plot some data on the axes.
    ax.plot(t, err_pitch, label='pitch')  # Plot more data on the axes...
    ax.plot(t, err_yaw, label='yaw')  # ... and some more.
    ax.set_xlabel('t [s]')  # Add an x-label to the axes.
    ax.set_ylabel('error [deg]')  # Add a y-label to the axes.
    ax.set_title("Simple Plot")  # Add a title to the axes.
    ax.legend()  # Add a legend.
    plt.show(block=True)
