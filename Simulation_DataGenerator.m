close all;
clear all;
addpath('Classes and Objects')
addpath('Functions')
addpath('Data Types')
%% Inputs
rng(0);
N = 10000;
dt = 0.01;   % in s
uwb_range = 20;
uwb_update_probability = 0.9; % determines avg frequency of uwb measurements
uwb_mode = rangingModes.tdoa;      % 'tdoa' or 'twr'

% Noise
noise.attitude = 0.05;
noise.uwb = 0.05;

% Constants
g = 9.81;
k_aero = 0.35;
%% Anchors
% remove anchor by comenting the line
i = 1;
% Cyberzoo-like setup
anchors(i) = Anchor(0,[-5,-5, 1.7]); i=i+1;
anchors(i) = Anchor(1,[-5, 5, 1.7]); i=i+1;
anchors(i) = Anchor(2,[ 5, 5, 0.9]); i=i+1;
anchors(i) = Anchor(3,[ 5,-5, 1.7]); i=i+1;
% Cube setup
anchors(i) = Anchor(4,[-5,-5, 0]); i=i+1;
anchors(i) = Anchor(5,[-5, 5, 0]); i=i+1;
anchors(i) = Anchor(6,[ 5, 5, 5]); i=i+1;
anchors(i) = Anchor(7,[ 5,-5, 0]); i=i+1;

%% Drone trajectory (ground truth)
data.time = (linspace(0,N-1,N)*dt)';
time = data.time;
%3rd order polynomial
% p0 = [1,2,0];
% p1 = [4,4,3];
% v0 = [1,5,0];
% v1 = [2,5,0];
% traj = trajSpline(p0,p1,v0,v1,time);

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
for i=2:N
    data.pitch(i) = atan(-(dt*data.vx(i)+k_aero*data.vx(i-1))/g);
    data.roll(i) = atan((dt*data.vy(i)+k_aero*data.vy(i-1))/g);

    data.pitch(i) = data.pitch(i)  + noise.attitude * randn();
    data.roll(i) = data.roll(i) + noise.attitude * randn();
    data.yaw(i) = data.yaw(i) + noise.attitude * randn();
end


%% UWB
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
