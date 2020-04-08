# MatlabUWB
This project was created to simulate different state estimators for drones using an UWB system for localization. The main file, `EstimatorSimulation_main.m` contains the simulation. The data for the simulation can come from two sources:
1. The `EstimatorSimulation_DataGenerator.m` script
2. Logs from a Crazyflie, brought into the same data format using the `Log2Data.m` script

To collect log data direclty from the crazyflie, you can use the `optitrackFlight.py` script in the Crazyflie folder. The script relies on the [Crazyflie Python library](https://github.com/bitcraze/crazyflie-lib-python/). 