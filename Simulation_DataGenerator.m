close all;
clear all;
addpath('Functions');
%% Inputs
rng(0);
N = 10000;
dt = 10;   % in ms like real drone data
uwb_range = 10;
uwb_update_probability = 0.05; % determines avg frequency of uwb measurements
uwb_mode = 'tdoa';      % 'tdoa' or 'twr'

% Noise
noise.attitude = 0.05;
noise.uwb = 0.05;

% Constants
g = 9.81;
k_aero = 0.35;
% Anchors (one row per anchor, [x,y,z]
anchors = [ 0, 0, 0
            0, 0, 5;
            0, 5, 0;
            0, 5, 5;
            5, 5, 0;
            5, 5, 5;
            5, 0, 0;
            5, 0, 5];

%% Drone trajectory (ground truth)
data.time = (linspace(0,N-1,N)*dt)';
time = data.time/1000; % in seconds for trajectory calculations
%3rd order polynomial
% p0 = [1,2,0];
% p1 = [4,4,3];
% v0 = [1,5,0];
% v1 = [2,5,0];
% traj = trajSpline(p0,p1,v0,v1,time);

% setpoints
points = [3, 3, 0;
          3, 3, 1;
          1, 1, 1;
          1, 4, 2;
          4, 4, 2;
          4, 1, 1;
          1, 1, 1];
traj = trajPoints(points, time);

data.x = traj(:,1);
data.y = traj(:,2);
data.z = traj(:,3);

data.vx = zeros(N,1);
data.vy = zeros(N,1);
data.vz = zeros(N,1);
for i=2:N
    data.vx(i) = (data.x(i)-data.x(i-1))/(dt/1000);
    data.vy(i) = (data.y(i)-data.y(i-1))/(dt/1000);
    data.vz(i) = (data.z(i)-data.z(i-1))/(dt/1000);
end

%% Drone attitude (reverse model)
data.pitch = zeros(N,1);
data.roll = zeros(N,1);
data.yaw = zeros(N,1);
for i=2:N
    data.pitch(i) = atan(-(dt*data.vx(i)+k_aero*data.vx(i-1))/g);
    data.roll(i) = atan((dt*data.vy(i)+k_aero*data.vy(i-1))/g);

    data.pitch(i) = data.pitch(i)  + noise.attitude * randn();
    data.roll(i) = data.roll(i) + noise.attitude * randn();
    data.yaw(i) = data.yaw(i) + noise.attitude * randn();
end


%% UWB
position = [data.x, data.y, data.z];
anch_inRange = anchorsInRange(anchors,position(1,:),uwb_range);
if (strcmp(uwb_mode,'twr'))
    data.uwb = getTWR(anch_inRange,position(1,:),time(1));
    data.uwb.distance = data.uwb.distance * (1 + noise.uwb * randn());
elseif (strcmp(uwb_mode,'tdoa'))
    data.uwb = getTDOA(anch_inRange,position(1,:),time(1));
    data.uwb.dist_diff = data.uwb.dist_diff * (1 + noise.uwb * randn());
end
        
for i=2:N
    if (rand()<=uwb_update_probability)
        anch_inRange = anchorsInRange(anchors,position(i,:),uwb_range);
        if (strcmp(uwb_mode,'twr'))
            data.uwb(i) = getTWR(anch_inRange,position(i,:),time(i));
            data.uwb(i).distance = data.uwb(i).distance * (1 + noise.uwb * randn());
        elseif (strcmp(uwb_mode,'tdoa'))
            data.uwb(i) = getTDOA(anch_inRange,position(i,:),time(i));
            data.uwb(i).dist_diff = data.uwb(i).dist_diff * (1 + noise.uwb * randn());
        end
    else
            data.uwb(i) = data.uwb(i-1);
    end
    
end

%% Save data
save("simulationData_test.mat","data");
