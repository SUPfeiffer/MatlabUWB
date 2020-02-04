%% Moving Horizon Estimator from Crazyflie logs
% input: data structure, fields: x,y,z,vx,vy,vz,pitch,roll,yaw
close all;
clear all;
addpath('Functions');
load('Data/square.mat')
%% Parameters
mode = 'multi';
pos_update_dt = 0.1;
mhe_timeHorizon = 0.5; % s
ransac_iter = 10;
ransac_sampleSize = 2;
ransac_outlierThreshold = 0.05;
PV = 1;
ransac_prior = [0 0; 
                0 PV];
            
MAX_WINDOW_SIZE = 100;
N = length(data.time);
const_g = 9.81;
const_k_aero = 0.35;

% initial values
prediction.p = [data.x(1),data.y(1),data.z(1)];
prediction.v = [0,0,0];
state.p = prediction.p;
state.v = prediction.v;
log.x(1) = state.p(1);
log.y(1) = state.p(2);
log.z(1) = state.p(3);
log.vx(1) = state.v(1);
log.vy(1) = state.v(2);
log.vz(1) = state.v(3);
last_uwb_timestamp = -1;
dpos_w = zeros(MAX_WINDOW_SIZE,3);
time_w = zeros(MAX_WINDOW_SIZE,1);
windowSize = 0;
corrector.p = [0,0,0];
corrector.v = [0,0,0];

% convert time to seconds
data.time = data.time/1000;
for i = 1:length(data.uwb)
    data.uwb(i).timestamp = data.uwb(i).timestamp/1000;
end

uwb_count = 1;

for i=291:N
    if(mod(data.time(i),pos_update_dt) == 0)
        % no estimation of attitude yet
        state.pitch = data.pitch(i);
        state.roll = data.roll(i);
        state.yaw = data.yaw(i);
    
        % z initial estimate not by mhe
        state.p(3) = data.z(i);
        state.v(3) = (data.z(i)-data.z(i-1))/(data.time(i)-data.time(i-1));
        
        % predict current state
        pitch_global = state.pitch * cos(state.yaw) - state.roll*sin(state.yaw);
        roll_global = state.pitch * sin(state.yaw) + state.roll*cos(state.yaw);
        
        prediction.p(1) = prediction.p(1) + prediction.v(1) * pos_update_dt;
        prediction.p(2) = prediction.p(2) + prediction.v(2) * pos_update_dt;
        prediction.p(3) = state.p(3);
        
        %prediction.v(1) = data.vx(i);
        %prediction.v(2) = data.vy(i);
        %prediction.v(3) = data.vz(i);
        
        prediction.v(1) = prediction.v(1) + (-const_g*tan(pitch_global) - const_k_aero*state.v(1))*pos_update_dt;
        prediction.v(2) = prediction.v(2) + (const_g*tan(roll_global) - const_k_aero*state.v(2))*pos_update_dt;
        prediction.v(3) = state.v(3);
        
        % Measurement
        measurement_update = 0;
        if(strcmp(mode,'single'))
            while (uwb_count <= length(data.uwb) && data.uwb(uwb_count).timestamp <= data.time(i))
                if (data.uwb(uwb_count).timestamp == data.time(i))
                    measurement_update = 1;
                    if (strcmp(data.uwb(uwb_count).mode,'tdoa'))
                        measurement = tdoa2position_s(data.uwb(uwb_count),prediction.p);
                    elseif(strcmp(data.uwb(uwb_count).mode,'twr'))
                        measurement = twr2position_s(data.uwb(uwb_count),prediction.p);
                    else
                        break;
                    end
                    log.measurement.x(uwb_count) = measurement(1);
                    log.measurement.y(uwb_count) = measurement(2);
                    log.measurement.z(uwb_count) = measurement(3);
                    % Window Updates
                    dpos_w = circshift(dpos_w,1,1);
                    time_w = circshift(time_w,1,1);
                    dpos_w(1,:) = measurement - prediction.p;
                    time_w(1) = data.time(i);

                    if (windowSize < MAX_WINDOW_SIZE)
                        windowSize = windowSize + 1;
                    end
                    while ((time_w(1)-time_w(windowSize)) > mhe_timeHorizon)
                        windowSize = windowSize -1;
                    end     
                end
                uwb_count = uwb_count + 1;
            end
        elseif(strcmp(mode,'multi'))
            while (uwb_count <= length(data.uwb) && data.uwb(uwb_count).timestamp <= data.time(i))
                newUWB = data.uwb(uwb_count);
                twr_mem(newUWB.id + 1) = newUWB;
                uwb_count = uwb_count + 1;     
            end
    
            measurement = twr2position_m(twr_mem(1),twr_mem(2),twr_mem(3),twr_mem(4));
            log.measurement.x(uwb_count) = measurement(1);
            log.measurement.y(uwb_count) = measurement(2);
            log.measurement.z(uwb_count) = measurement(3);
            % Window Updates
            dpos_w = circshift(dpos_w,1,1);
            time_w = circshift(time_w,1,1);
            dpos_w(1,:) = measurement - prediction.p;
            time_w(1) = data.time(i);

            if (windowSize < MAX_WINDOW_SIZE)
                windowSize = windowSize + 1;
            end
            while ((time_w(1)-time_w(windowSize)) > mhe_timeHorizon)
                windowSize = windowSize -1;
            end     
        else
            break;
        end

        
        
        if measurement_update
            dt_w = time_w - time_w(windowSize);

            if(windowSize >= ransac_sampleSize && sum(dt_w>0)>ransac_sampleSize-1)
                best_error = windowSize * ransac_outlierThreshold;

                for j=1:ransac_iter
                    idx = randsample(windowSize,ransac_sampleSize);

                    if (ransac_sampleSize == 2)
                        denom = (dt_w(idx(2)) - dt_w(idx(1)))^2 + 2*PV;

                        s_dp = (( dpos_w(idx(1),:) * dt_w(idx(2)) - dpos_w(idx(2),:) * dt_w(idx(1)) ) ...
                                * (dt_w(idx(2)) - dt_w(idx(1))) + (dpos_w(idx(1),:) + dpos_w(idx(2),:)) * PV)/denom;
                        s_dv = ( dpos_w(idx(2),:) - dpos_w(idx(1),:) ) * (dt_w(idx(2)) - dt_w(idx(1))) / denom;
                    else
                        [s_dp(1),s_dv(1)] = linearLeastSquares(dt_w(idx),dpos_w(idx,1),ransac_prior);
                        [s_dp(2),s_dv(2)] = linearLeastSquares(dt_w(idx),dpos_w(idx,2),ransac_prior);
                        [s_dp(3),s_dv(3)] = linearLeastSquares(dt_w(idx),dpos_w(idx,3),ransac_prior);
                    end

                    step_error = s_dv.*dt_w(1:windowSize) + s_dp - dpos_w(1:windowSize,:);
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
                    dpos_in = dpos_w(~best_isOutlier,:);
                    dt_in = dt_w(~best_isOutlier);

                    [corrector.p(1), corrector.v(1)] = linearLeastSquares(dt_in,dpos_in(:,1),ransac_prior);
                    [corrector.p(2), corrector.v(2)] = linearLeastSquares(dt_in,dpos_in(:,2),ransac_prior);
                    [corrector.p(3), corrector.v(3)] = linearLeastSquares(dt_in,dpos_in(:,3),ransac_prior);
                else
                    corrector.p = best_dp;
                    corrector.v = best_dv;
                end

            end

        end
        if windowSize > 0
            delta_t = (data.time(i) - time_w(windowSize))/1000;
        else
            delta_t = data.time(i);
        end
        state.p = prediction.p + corrector.p + corrector.v*delta_t;
        state.v = prediction.v + corrector.v;
    end
    
    log.time(i) = data.time(i);
    log.x(i) = state.p(1);
    log.y(i) = state.p(2);
    log.z(i) = state.p(3);
    log.vx(i) = state.v(1);
    log.vy(i) = state.v(2);
    log.vz(i) = state.v(3);

end

%% Plots
% trajectory
figure(1)
subplot(2,2,1)
axis([0,5,0,5,0,5])
xticks([0,1,2,3,4,5])
xlabel('x')
yticks([0,1,2,3,4,5])
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
axis([0,5,0,5,])
xticks([0,1,2,3,4,5])
xlabel('x')
yticks([0,1,2,3,4,5])
ylabel('y')
daspect([1 1 1])
hold('on')
grid('on')
grid minor
plot(data.x,data.y,'b')
plot(log.x,log.y,'r')
% x-z
subplot(2,2,3)
axis([0,5,0,5])
xticks([0,1,2,3,4,5])
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
axis([0,5,0,5])
xticks([0,1,2,3,4,5])
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