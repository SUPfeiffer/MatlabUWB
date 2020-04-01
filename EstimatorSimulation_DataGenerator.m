%% Estimator Simulation: Data Generator
% This script uses a strongly simplified drone model to create a dataset
% that can be used to simulate different state estimators with the
% EstimatorSimulation_main.m script.
%
% Author: S. Pfeiffer, MAVLab TU Delft


%% Setup
% Add paths
addpath('Functions')
addpath('Classes')

% close figures and clear workspace
close all;
clear all;

% Constants
g = 9.81;
k_aero = 0.35;

% random number generator seed
rng(0);

% Simulation parameters
N = 10000;   % data points
dt = 0.01;   % timestep in s

% UWB parameters
uwb_range = 20;                 % not used atm
uwb_update_probability = 0.7;   % determines avg frequency of uwb measurements
uwb_mode = rangingModes.tdoa;   % 'tdoa' or 'twr'

% Noise stdev
noise.attitude = 0.05;
noise.uwb = 0.1;

%% UWB Anchors
% remove anchor by comenting the line
i = 1;
% Cyberzoo-like setup
anchors(i) = Anchor(0,[-5,-5, 1.7]); i=i+1;
anchors(i) = Anchor(1,[-5, 5, 1.7]); i=i+1;
anchors(i) = Anchor(2,[ 5, 5, 0.9]); i=i+1;
anchors(i) = Anchor(3,[ 5,-5, 1.7]); i=i+1;
% Cube setup
%anchors(i) = Anchor(4,[-5,-5, 0]); i=i+1;
%anchors(i) = Anchor(5,[-5, 5, 0]); i=i+1;
%anchors(i) = Anchor(6,[ 5, 5, 5]); i=i+1;
%anchors(i) = Anchor(7,[ 5,-5, 0]); i=i+1;

%% Drone trajectory (ground truth)
data.time = (linspace(0,N-1,N)*dt)';
time = data.time;

% setpoints
points = [ 0, 0, 0;
           0, 0, 2;
          -3,-3, 2;
          -3, 3, 2;
           3, 3, 2;
           3,-3, 2;
          -3,-3, 2;
           0, 0, 2;
           0, 0, 0];
traj = trajPoints(points, time);

data.x = traj(:,1);
data.y = traj(:,2);
data.z = traj(:,3);

data.vx = zeros(N,1);
data.vy = zeros(N,1);
data.vz = zeros(N,1);
for i=2:N
    data.vx(i) = (data.x(i)-data.x(i-1))/dt;
    data.vy(i) = (data.y(i)-data.y(i-1))/dt;
    data.vz(i) = (data.z(i)-data.z(i-1))/dt;
end

%% Drone attitude (reverse model)
data.pitch = zeros(N,1);
data.roll = zeros(N,1);
data.yaw = zeros(N,1);
for i=1:N-1
    data.pitch(i) = atan(((data.vx(i+1)-data.vx(i))/dt + k_aero*data.vx(i))/g);
    data.roll(i) = atan(-((data.vy(i+1)-data.vy(i))/dt + k_aero*data.vy(i))/g);

    data.pitch(i) = data.pitch(i)  + noise.attitude * randn();
    data.roll(i) = data.roll(i) + noise.attitude * randn();
    data.yaw(i) = data.yaw(i) + noise.attitude * randn();
end


%% Calculate UWB measurements
position = [data.x, data.y, data.z];
uwb_count = 1;
for i=1:N
    for j=1:length(anchors)
        if (rand()<=uwb_update_probability)
            if uwb_mode == rangingModes.twr
                dist = norm(position(i,:)-anchors(j).position) + noise.uwb * randn();
                data.uwb(uwb_count) = TWRPacket(anchors(j),dist,data.time(i));
                uwb_count = uwb_count + 1;
            elseif uwb_mode == rangingModes.tdoa
                anch2idx = randi(length(anchors));
                while anch2idx == j
                    anch2idx = randi(length(anchors));
                end
                dA = norm(position(i,:)-anchors(j).position);
                dB = norm(position(i,:)-anchors(anch2idx).position);
                dist = dA - dB + noise.uwb * randn();
                data.uwb(uwb_count) = TDOAPacket(anchors(j),anchors(anch2idx),dist,data.time(i));
                uwb_count = uwb_count + 1;
            end
        end
    end
end

%% Save data
save("simulationData_test.mat","data");
