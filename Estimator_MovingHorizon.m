%% Moving Horizon Estimator from Crazyflie logs
% input: data structure, fields: x,y,z,vx,vy,vz,pitch,roll,yaw
close all;
%clear all;
addpath('Functions');
addpath('Classes');
%load('Data/square.mat')
load('simulationData_test.mat');
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
        % Prediction 
        pitch_global = data.pitch(i) * cos(data.yaw(i)) - data.roll(i)*sin(data.yaw(i));
        roll_global = data.pitch(i) * sin(data.yaw(i)) + data.roll(i)*cos(data.yaw(i));
        
        state_prediction.p(1) = state_prediction.p(1) + state_prediction.v(1) * pos_update_dt;
        state_prediction.p(2) = state_prediction.p(2) + state_prediction.v(2) * pos_update_dt;
        
        state_prediction.v(1) = state_prediction.v(1)...
                                + (-g*tan(pitch_global) - k_aero*state_prediction.v(1)) * pos_update_dt;
        state_prediction.v(2) = state_prediction.v(2)...
                                + (g*tan(roll_global) - k_aero*state_prediction.v(2)) * pos_update_dt;
        log.prediction(i,:) = state_prediction.p; 
        
        % Measurement
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
        
        measurement = uwbMultilat(uwb_mem, 'prior', state_estimate.p,'nbest',10);%, 'forceZ', data.z(i));
        
        %%%%%%%%%%%
        % Single Measurement Projection & Variations
%         prior = state_estimate.p;
%         while (uwb_count <= length(data.uwb) && data.uwb(uwb_count).timestamp <= time_now)
%             prior = uwbProject(data.uwb(uwb_count), prior,'forceZ',data.z(i));
%             uwb_count = uwb_count + 1;
%         end
%         measurement = prior;
        %%%%%%%%%%%
        
        log.measurement.x(i) = measurement(1);
        log.measurement.y(i) = measurement(2);
        log.measurement.z(i) = measurement(3);
        
        if measurement_update
            % Update moving windows
            wPrediction = [state_prediction.p; wPrediction(1:end-1,:)];
            wMeasurement = [measurement; wMeasurement(1:end-1,:)];
            wTime = [time_now; wTime(1:end-1)];
            
            if (windowSize < MAX_WINDOW_SIZE)
                windowSize = windowSize + 1;
            end
            while ((wTime(1)-wTime(windowSize)) > mhe_timeHorizon)
                windowSize = windowSize -1;
            end
            
            % calculate delta windows
            wDt = wTime - wTime(windowSize);
            wDp = wMeasurement - wPrediction;

            % RANSAC MHE
            if(windowSize >= ransac_sampleSize && sum(wDt>0)>ransac_sampleSize-1)
                best_error = windowSize * ransac_outlierThreshold + 1;

                for j=1:ransac_iter
                    idx = randsample(windowSize,ransac_sampleSize);

                    if (ransac_sampleSize == 2)
                        denom = (wDt(idx(2)) - wDt(idx(1)))^2 + 2*PV;

                        s_dp = (( wDp(idx(1),:) * wDt(idx(2)) - wDp(idx(2),:) * wDt(idx(1)) ) ...
                                * (wDt(idx(2)) - wDt(idx(1))) + (wDp(idx(1),:) + wDp(idx(2),:)) * PV)/denom;
                        s_dv = ( wDp(idx(2),:) - wDp(idx(1),:) ) * (wDt(idx(2)) - wDt(idx(1))) / denom;
                    else
                        [s_dp(1),s_dv(1)] = linearLeastSquares(wDt(idx),wDp(idx,1),ransac_prior);
                        [s_dp(2),s_dv(2)] = linearLeastSquares(wDt(idx),wDp(idx,2),ransac_prior);
                        [s_dp(3),s_dv(3)] = linearLeastSquares(wDt(idx),wDp(idx,3),ransac_prior);
                    end

                    step_error = s_dv.*wDt(1:windowSize) + s_dp - wDp(1:windowSize,:);
                    step_error = sqrt(sum(step_error.^2,2)); % mean squared error for each timestep
                    isOutlier = (step_error>ransac_outlierThreshold); % identify outliers
                    step_error(isOutlier) = ransac_outlierThreshold; % prune outlier error
                    total_error = sum(step_error);

                    % Update best model
                    if (total_error < best_error)
                       best_error = total_error;
                       best_isOutlier = isOutlier;
                       best_dp = s_dp;
                       best_dv = s_dv;
                    end
                end

                % Recalculate best model from inliers
                if (sum(~best_isOutlier)>=2)
                    dpos_in = wDp(~best_isOutlier,:);
                    dt_in = wDt(~best_isOutlier);

                    [state_corrector.p(1), state_corrector.v(1)] = linearLeastSquares(dt_in,dpos_in(:,1),ransac_prior);
                    [state_corrector.p(2), state_corrector.v(2)] = linearLeastSquares(dt_in,dpos_in(:,2),ransac_prior);
                    [state_corrector.p(3), state_corrector.v(3)] = linearLeastSquares(dt_in,dpos_in(:,3),ransac_prior);
                else
                    state_corrector.p = best_dp;
                    state_corrector.v = best_dv;
                end

            end

        end
        if windowSize > 0
            delta_t = (data.time(i) - wTime(windowSize))/1000;
        else
            delta_t = data.time(i);
        end
        state_estimate.p = state_prediction.p + state_corrector.p + state_corrector.v*delta_t;
        state_estimate.v = state_prediction.v + state_corrector.v;
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
%plot3(log.x,log.y,log.z,'r')