%% Kidnapping simulation with MHE & TDOA projection (2D)
close all;
clear all;
addpath('Functions');
%% Inputs
rng(0);
N = 10001;
f_m = 10; % measurement frequency
dt = 0.01;
uwb_range = 10;

MODE_UWB = 'tdoa';      % 'tdoa' or 'twr'
MODE_2POS = 'S';        % 'S'(ingle) or 'M'(ultiple)
MODE_OUTLIER = 'RANSAC';% 'RANSAC' or 'LS'

MAX_WINDOW_SIZE = 10;
RANSAC_ITER = 5;
ERROR_TH = 0.2;
PV = 1; % penalty factor for velocity corrector

P = [0, 0; 0, PV]; % penalty matrix

% Anchors (one row per anchor, [x,y,z]
anchors = [ 0, 0, 0
            0, 0, 5;
            0, 5, 0;
            0, 5, 5;
            5, 5, 0;
            5, 5, 5;
            5, 0, 0;
            5, 0, 5];

% Tag trajectory (ground truth)
time = linspace(0,N-1,N)*dt;

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

real(1).p = traj(1,:);
for i=2:N
    real(i).p = traj(i,:);
    real(i-1).v = (real(i).p-real(i-1).p)/dt;
end

% Tag estimator
pred.p = [3,3,0];
pred.v = [0,0,0];

state.p = pred.p;
state.v = pred.v;

corrector.p = [0,0,0];
corrector.v = [0,0,0];
corrector.t = 0;


% Noise
noise.v = [0.02, 0.02, 0.02];
noise.uwb = 0.05;

%% Simulation
j = 1; % tdoa counter
windowSize = 0;
for i=2:N-1
    % Prediction Update
    pred(i).p = pred(i-1).p + pred(i-1).v * dt;
    pred(i).v = (noise.v .* randn(1,3)) + real(i).v;
    
    if (mod(i,f_m)==0)
        
        
        %% get uwb measurement
        anch_inRange = anchorsInRange(anchors,real(i).p,uwb_range);
        if (strcmp(MODE_UWB,'twr'))
            uwb(j) = getTWR(anch_inRange,real(i).p,time(i));
            uwb(j).distance = (1 + noise.uwb * randn())*uwb(j).distance;    
        elseif (strcmp(MODE_UWB,'tdoa'))
            uwb(j) = getTDOA(anch_inRange,real(i).p,time(i));
            uwb(j).dist_diff = (1 + noise.uwb * randn())*uwb(j).dist_diff;
        else
            sprintf("'%s' is not a valid MODE_UWB. Valid modes are 'tdoa' and 'twr'",MODE_UWB) 
            break;
        end
        
        %% get position estimate
        if (strcmp(MODE_2POS,'S'))
            prior_p = pred(i).p + corrector(j).p + corrector(j).v * (time(i)-corrector(j).t);
            if (strcmp(uwb(j).mode,'tdoa'))
                meas(j).p = tdoa2position_s(uwb(j),prior_p); % should use corrected prediction instead of state
            else
                meas(j).p = twr2position_s(uwb(j),prior_p);
            end            
        elseif (strcmp(MODE_2POS, 'M'))
            if (strcmp(uwb(j).mode, 'tdoa'))
                
            else
                
            end            
        else
           sprintf("'%s' is not a valid MODE_2POS. Valid modes are 'S' and 'M'")
           break;            
        end
            
                
        %% update measurement windows
        if (windowSize < MAX_WINDOW_SIZE)
            % increase window size
            windowSize = windowSize+1;
            time_w(windowSize,:) = 0;
            dpos_w(windowSize,:) = [0,0,0];
        end
        
        time_w = circshift(time_w,1);
        dpos_w = circshift(dpos_w,1);
        
        time_w(1) = time(i);
        dpos_w(1,:) = meas(j).p - pred(i).p;
        
        
        dt_w = time_w - time_w(windowSize);
        dt_w = dt_w .* (dt_w>0);
        
        corrector(j+1).t = time_w(windowSize); % timestamp the model
        if (windowSize >= 2)
            % RANSAC (2 Points)
            best_error = (windowSize+1) * ERROR_TH;
            for k=1:RANSAC_ITER
                % calculate corrector model for sample (2 Points)
                idx1 = randi(windowSize);
                idx2 = randi(windowSize);
                while(idx1==idx2)
                    idx2 = randi(windowSize);
                end
                denom = (dt_w(idx2) - dt_w(idx1))^2 + 2*PV;
                
                s_dp = (( dpos_w(idx1,:) * dt_w(idx2) - dpos_w(idx2,:) * dt_w(idx1) ) ...
                        * (dt_w(idx2) - dt_w(idx1)) + (dpos_w(idx1,:) + dpos_w(idx2,:)) * PV)/denom;
                s_dv = ( dpos_w(idx2,:) - dpos_w(idx1,:) ) * (dt_w(idx2) - dt_w(idx1)) / denom;
                
                %s_dp = ( dpos_w(idx1,:) * dt_w(idx2) ...
                %             -dpos_w(idx2,:) * dt_w(idx1) )/(dt_w(idx2)-dt_w(idx1));
                
                %s_dv = (dpos_w(idx2,:) - dpos_w(idx1,:)) / (dt_w(idx2)-dt_w(idx1));
                
                % Calculate total error
                step_error = s_dv.*dt_w + s_dp - dpos_w;
                step_error = sqrt(sum(step_error.^2,2)); % mean squared error for each timestep
                isOutlier = (step_error>ERROR_TH); % identify outliers
                step_error(isOutlier) = ERROR_TH; % prune outlier error
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
                dpos_in = dpos_w .* ~best_isOutlier;
                dt_in = dt_w .* ~best_isOutlier;
                count_in = sum(~best_isOutlier);

                [corrector(j+1).p(1), corrector(j+1).v(1)] = ...
                            linearLeastSquares(dt_in,dpos_in(:,1),P);
                [corrector(j+1).p(2), corrector(j+1).v(2)] = ...
                            linearLeastSquares(dt_in,dpos_in(:,2),P);
                [corrector(j+1).p(3), corrector(j+1).v(3)] = ...
                            linearLeastSquares(dt_in,dpos_in(:,3),P);
            else
                corrector(j+1).p = best_dp;
                corrector(j+1).v = best_dv;
            end
        else
            corrector(j+1).p =[0, 0, 0];
            corrector(j+1).v = [0, 0, 0];
        end
        j = j+1;
    end
    
    state(i).p = pred(i).p + corrector(j).p + corrector(j).v * (time(i)-corrector(j).t);
    state(i).v = pred(i).v + corrector(j).v;
    
    error.p(i,:) = state(i).p - real(i).p;
    error.v(i,:) = state(i).v - real(i).v;
end

%% plots
for i=1:N-1
    p_realx(i) = real(i).p(1);
    p_realy(i) = real(i).p(2);
    p_realz(i) = real(i).p(3);
    p_predx(i) = pred(i).p(1);
    p_predy(i) = pred(i).p(2);
    p_predz(i) = pred(i).p(3);
    p_statex(i) = state(i).p(1);
    p_statey(i) = state(i).p(2);
    p_statez(i) = state(i).p(3);
end
for i=1:j-1
    p_measx(i)=meas(i).p(1);
    p_measy(i)=meas(i).p(2);
    p_measz(i)=meas(i).p(3);
end
   
% 3D position
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
plot3(p_realx,p_realy,p_realz,'b')
plot3(p_statex,p_statey,p_statez,'r')
%plot3(p_measx,p_measy,p_measz,'+k')
plot3(anchors(:,1),anchors(:,2),anchors(:,3),'*m')
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
plot(p_realx,p_realy,'b')
plot(p_statex,p_statey,'r')
%plot(p_measx,p_measy,'+k')
plot(anchors(:,1),anchors(:,2),'*m')
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
plot(p_realx,p_realz,'b')
plot(p_statex,p_statez,'r')
%plot(p_measx,p_measz,'+k')
plot(anchors(:,1),anchors(:,3),'*m')
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
plot(p_realy,p_realz,'b')
plot(p_statey,p_statez,'r')
%plot(p_measy,p_measz,'+k')
plot(anchors(:,2),anchors(:,3),'*m')

% errors
figure(2)
subplot(2,1,1)
hold on
grid on
grid minor
plot(time(1:end-1),error.p(:,1));
plot(time(1:end-1),error.p(:,2));
plot(time(1:end-1),error.p(:,3));
legend('x','y','z')
subplot(2,1,2)
hold on
grid on
grid minor
plot(time(1:end-1),error.v(:,1));
plot(time(1:end-1),error.v(:,2));
plot(time(1:end-1),error.v(:,3));
legend('x','y','z')
%fileName = sprintf('SIM_%s-%s_%s',mode_UWB,mode_uwb2pos,mode_outlierRej);
