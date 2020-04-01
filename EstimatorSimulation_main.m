%% Estimator Simulation
% This script simulates different estimators that are included as systems.
% Input data can be loaded from the following sources:
%     - Generated data: Data can be generated with the
%       EstimatorSimulation_DataGenerator.m script.
%     - Crazyflie logs: Collect Data with the provided python script and
%       convert it using the Log2Data.m script.
% To improve performance when running several simulations with the same
% dataset, comment the lines 
% 
% Author: S. Pfeiffer, MAVLab TU Delft

%% Setup
% add paths to external functions, classes and systems
addpath('Functions');
addpath('Classes');
addpath('Systems');

% close open figures
close all;

% clear workspace and load simulation Data. Comment to increase speed if
% running the script with the same data several times.
clear all;
load('Data/square.mat');

% Simulation parameters
pos_update_dt = 0.01;
N = length(data.time);

% Initialize estimator (Choose one)
estimator = MovingHorizonEstimator('timeHorizon', 0.15,...
                                   'ransacIterations',10,...
                                   'ransacSamples', 2,...
                                   'ransacPrior', [0, 0; 0, 10],...
                                   'outlierThreshold', 0.1);       
% estimator = adaptiveUFIR('dt', pos_update_dt);

% initialize custom variable types "QuadrotorState"
state_prediction = QuadrotorState();
state_corrector = QuadrotorState();
state_estimate = QuadrotorState();
measurement = QuadrotorState();

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

% UWB mode from data
mode = data.uwb(1).rangingMode;

%% Simulation
uwb_count = 1;
for i=10:N  % start at 10 so that there is data available for all variables
    time_now = data.time(i);
    if(mod(time_now,pos_update_dt) == 0)
        %% Measurement
        % This section implements to possible approaches to collect
        % position measurements, either by using multilateration with
        % several beacons, or using projection strategies with single
        % beacons.
        % TODO: implement a coherent form that does not rely on
        % commenting/uncommenting code. Ideally, only pass tdoa/twr
        % raw measurements to estimators, but that requires more
        % substantial changes to the estimator systems.
        
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
        
        measurement.p = uwbMultilat(uwb_mem, 'prior', state_estimate.p,'nbest',10, 'forceZ', data.z(i));
        measurement.attitude(1) = data.roll(i);        
        measurement.attitude(2) = data.pitch(i);
        measurement.attitude(3) = data.yaw(i);
        measurement.timestamp = time_now;
        
%         %%%%%%%%%%%
%         % Single Measurement Projection & Variations
%         prior = state_estimate.p;
%         while (uwb_count <= length(data.uwb) && data.uwb(uwb_count).timestamp <= time_now)
%             prior = uwbProject(data.uwb(uwb_count), prior,'forceZ',data.z(i));
%             uwb_count = uwb_count + 1;
%         end
%         measurement.p = prior;
%         measurement.attitude(1) = data.roll(i);        
%         measurement.attitude(2) = data.pitch(i);
%         measurement.attitude(3) = data.yaw(i);
%         measurement.timestamp = time_now;
%         %%%%%%%%%%%
        
        log.measurement.x(i) = measurement.p(1);
        log.measurement.y(i) = measurement.p(2);
        log.measurement.z(i) = measurement.p(3);
        
        [state_prediction,state_estimate] = estimator(measurement,time_now);

       

    end
    
    % Add state variables to the log
    log.time(i) = data.time(i);
    log.p_x(i) = state_prediction.p(1);
    log.p_y(i) = state_prediction.p(2);
    log.p_z(i) = state_prediction.p(3);
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
axis([-5,5,-5,5,-5,0])
xticks(-5:5)
xlabel('x')
yticks(-5:5)
ylabel('y')
zticks(-5:0)
zlabel('z')
daspect([1 1 1])
hold('on')
grid('on')
grid minor
plot3(data.x,data.y,data.z,'b')
plot3(log.x,log.y,log.z,'r')
plot3(log.p_x,log.p_y,log.p_z,'g')
set(gca, 'YDir','reverse')
set(gca, 'ZDir','reverse')
% x-y
subplot(2,2,2)
axis([-5,5,-5,5])
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
plot(log.p_x,log.p_y,'g')
set(gca, 'YDir','reverse')
% x-z
subplot(2,2,3)
axis([-5,5,-5,0])
xticks(-5:5)
xlabel('x')
yticks(-5:0)
ylabel('z')
daspect([1 1 1])
hold('on')
grid('on')
grid minor
plot(data.x,data.z,'b')
plot(log.x,log.z,'r')
plot(log.p_x,log.p_z,'g')
set(gca, 'YDir','reverse')
%  y-z
subplot(2,2,4)
axis([-5,5,-5,0])
xticks(-5:5)
xlabel('y')
yticks(-5:0)
ylabel('z')
daspect([1 1 1])
hold('on')
grid('on')
grid minor
plot(data.y,data.z,'b')
plot(log.y,log.z,'r')
plot(log.p_y,log.p_z,'g')
set(gca, 'XDir','reverse')
set(gca, 'YDir','reverse')

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
axis([-5,5,-5,5,-5,0])
xticks(-5:5)
xlabel('x')
yticks(-5:5)
ylabel('y')
zticks(-5:0)
zlabel('z')
daspect([1 1 1])
hold('on')
grid('on')
grid minor
plot3(data.x,data.y,data.z,'b')
plot3(log.measurement.x, log.measurement.y, log.measurement.z ,'x')
plot3(log.x,log.y,log.z,'r')
set(gca, 'YDir','reverse')
set(gca, 'ZDir','reverse')

% figure(4)
% plot(estimator.wy(:,1),'r');
% hold on
% plot(estimator.wy(:,3),'b');
% plot(estimator.wy(:,2),'g');
% plot(estimator.x(:,1),'r--');
% plot(estimator.x(:,5),'b--');
% plot(estimator.x(:,3),'g--');