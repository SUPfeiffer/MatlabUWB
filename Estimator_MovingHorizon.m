%% Moving Horizon Estimator from Crazyflie logs
% input: data structure, fields: x,y,z,vx,vy,vz,pitch,roll,yaw
close all;
%clear all;
addpath('Functions');
addpath('Classes');
addpath('Systems');
%load('Data/square.mat')
%load('simulationData_test.mat');
%% Parameters
pos_update_dt = 0.01;
mhe_timeHorizon = 0.5; % s
ransac_iter = 10;
ransac_sampleSize = 2;
ransac_outlierThreshold = 0.05;
PV = 10;
ransac_prior = [0 0; 
                0 PV];
            
MAX_WINDOW_SIZE = 100;
N = length(data.time);
k_aero = 0.35;
g = 9.81;

% initialize objects
state_prediction = QuadrotorState();
state_corrector = QuadrotorState();
state_estimate = QuadrotorState();
measurement = QuadrotorState();
estimator = MovingHorizonEstimator();

% initial values
pos0 = [data.x(1),data.y(1),data.z(1)];
state_prediction.p = pos0;
state_estimate.p = pos0;
predictor.p0 = pos0;

log.x(1) = state_estimate.p(1);
log.y(1) = state_estimate.p(2);
log.z(1) = state_estimate.p(3);
log.vx(1) = state_estimate.v(1);
log.vy(1) = state_estimate.v(2);
log.vz(1) = state_estimate.v(3);

% MHE windows
wPrediction = zeros(MAX_WINDOW_SIZE,3);
wMeasurement = zeros(MAX_WINDOW_SIZE,3);
wTime = zeros(MAX_WINDOW_SIZE,1);
windowSize = 0;

uwb_count = 1;
mode = data.uwb(1).rangingMode;

%% Simulation
for i=10:N
    time_now = data.time(i);
    if(mod(time_now,pos_update_dt) == 0)
        %% Measurement
        measurement_update = 0;
        %%%%%%%%%%%
        % Multilateration & Variations
        if mode == rangingModes.twr
            while (uwb_count <= length(data.uwb) && data.uwb(uwb_count).timestamp <= time_now)
                uwb_mem(data.uwb(uwb_count).anchorID + 1) = data.uwb(uwb_count);
                uwb_count = uwb_count + 1;
                measurement_update = 1;
            end
            
        elseif mode == rangingModes.tdoa

            while (uwb_count <= length(data.uwb) && data.uwb(uwb_count).timestamp <= time_now)
                idxA = data.uwb(uwb_count).anchorID_A + 1;
                idxB = data.uwb(uwb_count).anchorID_B + 1;
                if idxA>idxB
                    data.uwb(uwb_count).switchAnchors();
                    idxA = data.uwb(uwb_count).anchorID_A + 1;
                    idxB = data.uwb(uwb_count).anchorID_B + 1;
                end
                uwb_mat(idxA,idxB) = data.uwb(uwb_count);
                uwb_logical(idxA,idxB) = true;
                uwb_count = uwb_count + 1;
                measurement_update = 1;
            end
            uwb_mem = uwb_mat(uwb_logical);
        end
        
        measurement.p = uwbMultilat(uwb_mem, 'prior', state_estimate.p,'nbest',10);%, 'forceZ', data.z(i));
        measurement.pitch = data.pitch(i);
        measurement.roll = data.roll(i);
        measurement.yaw = data.yaw(i);
        measurement.timestamp = time_now;
        
        %%%%%%%%%%%
        % Single Measurement Projection & Variations
%         prior = state_estimate.p;
%         while (uwb_count <= length(data.uwb) && data.uwb(uwb_count).timestamp <= time_now)
%             prior = uwbProject(data.uwb(uwb_count), prior,'forceZ',data.z(i));
%             uwb_count = uwb_count + 1;
%         end
%         measurement = prior;
        %%%%%%%%%%%
        
        log.measurement.x(i) = measurement.p(1);
        log.measurement.y(i) = measurement.p(2);
        log.measurement.z(i) = measurement.p(3);
        
        state_estimate = estimator(measurement,time_now);

    end
    
    log.time(i) = data.time(i);
    log.x(i) = state_estimate.p(1);
    log.y(i) = state_estimate.p(2);
    log.z(i) = state_estimate.p(3);
    log.vx(i) = state_estimate.v(1);
    log.vy(i) = state_estimate.v(2);
    log.vz(i) = state_estimate.v(3);

end

%% Plots
% trajectory
figure(1)
subplot(2,2,1)
axis([-5,5,-5,5,0,5])
xticks(-5:5)
xlabel('x')
yticks(-5:5)
ylabel('y')
zticks([0,1,2,3,4,5])
zlabel('z')
daspect([1 1 1])
hold('on')
grid('on')
grid minor
plot3(data.x,data.y,data.z,'b')
plot3(log.x,log.y,log.z,'r')
% x-y
subplot(2,2,2)
axis([-5,5,-5,5,])
xticks(-5:5)
xlabel('x')
yticks(-5:5)
ylabel('y')
daspect([1 1 1])
hold('on')
grid('on')
grid minor
plot(data.x,data.y,'b')
plot(log.x,log.y,'r')
% x-z
subplot(2,2,3)
axis([-5,5,0,5])
xticks(-5:5)
xlabel('x')
yticks([0,1,2,3,4,5])
ylabel('z')
daspect([1 1 1])
hold('on')
grid('on')
grid minor
plot(data.x,data.z,'b')
plot(log.x,log.z,'r')
%  y-z
subplot(2,2,4)
axis([-5,5,0,5])
xticks(-5:5)
xlabel('y')
yticks([0,1,2,3,4,5])
ylabel('z')
daspect([1 1 1])
hold('on')
grid('on')
grid minor
plot(data.y,data.z,'b')
plot(log.y,log.z,'r')

% errors
error.x = log.x - data.x';
error.y = log.y - data.y';
error.z = log.z - data.z';
%error.vx = log.vx - data.vx';
%error.vy = log.vy - data.vy';
%error.vz = log.vz - data.vz';

figure(2)
%subplot(2,1,1)
hold on
grid on
grid minor
plot(log.time,error.x);
plot(log.time,error.y);
plot(log.time,error.z);
legend('x','y','z')
%subplot(2,1,2)
%hold on
%grid on
%grid minor
%plot(log.time,error.vx);
%plot(log.time,error.vy);
%plot(log.time,error.vz);
%legend('x','y','z')

figure(3)
axis([-5,5,-5,5,-5,5])
xticks([-5,-4,-3,-2,-1,0,1,2,3,4,5])
xlabel('x')
yticks([-5,-4,-3,-2,-1,0,1,2,3,4,5])
ylabel('y')
zticks([-5,-4,-3,-2,-1,0,1,2,3,4,5])
zlabel('z')
daspect([1 1 1])
hold('on')
grid('on')
grid minor
plot3(data.x,data.y,data.z,'b')
plot3(log.measurement.x, log.measurement.y, log.measurement.z ,'x')
plot3(log.x,log.y,log.z,'r')