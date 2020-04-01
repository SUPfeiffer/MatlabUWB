classdef MovingHorizonEstimator < matlab.System
    % MovingHorizonEstimator
    %
    % FIR state estimator based on S.Li 2019
    % (https://arxiv.org/abs/1905.10110)

    % Public, tunable properties
    properties
        % MHE
        maxWindowSize = 100;
        timeHorizon = 0.15;
        % RANSAC
        ransacIterations = 10;
        ransacSamples = 2;
        ransacPrior = [0, 0; 0, 50];
        outlierThreshold = 0.5;
    end

    properties(DiscreteState)
        x_pred
        v_pred
        t_pred
        dx_est
        dv_est
        last_error_update
        wt
        wDp
        windowSize
    end

    % Pre-computed constants
    properties(Access = private)
        initialState = QuadrotorState();
        % Model
        g = 9.81;   % gravitational constant
        k = 0.35;   % drag coefficient
    end

    methods
        function obj = MovingHorizonEstimator(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:})
        end
    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
        end

        function [pred, est] = stepImpl(obj, measurement, time)
            % Prediction
            obj.updatePrediction(measurement.attitude, time);
            obj.x_pred(3) = measurement.p(3);
            
            if measurement.timestamp > obj.last_error_update
                wDt = obj.updateWindows(measurement, time);
                if(obj.windowSize >= obj.ransacSamples && sum(wDt>0)>obj.ransacSamples-1)
                    best_error = obj.windowSize * obj.outlierThreshold + 1;

                    for i=1:obj.ransacIterations
                        idx = randsample(obj.windowSize,obj.ransacSamples);

                        if (obj.ransacSamples == 2)
                            PV = obj.ransacPrior(2,2);
                            denom = (wDt(idx(2)) - wDt(idx(1)))^2 + 2*PV;

                            s_dp = (( obj.wDp(idx(1),:) * wDt(idx(2)) - obj.wDp(idx(2),:) * wDt(idx(1)) ) ...
                                    * (wDt(idx(2)) - wDt(idx(1))) + (obj.wDp(idx(1),:) + obj.wDp(idx(2),:)) * PV)/denom;
                            s_dv = ( obj.wDp(idx(2),:) - obj.wDp(idx(1),:) ) * (wDt(idx(2)) - wDt(idx(1))) / denom;
                        else
                            [s_dp(1),s_dv(1)] = linearLeastSquares(wDt(idx),obj.wDp(idx,1),obj.ransacPrior);
                            [s_dp(2),s_dv(2)] = linearLeastSquares(wDt(idx),obj.wDp(idx,2),obj.ransacPrior);
                            [s_dp(3),s_dv(3)] = linearLeastSquares(wDt(idx),obj.wDp(idx,3),obj.ransacPrior);
                        end

                        step_error = s_dv.*wDt(1:obj.windowSize) + s_dp - obj.wDp(1:obj.windowSize,:);
                        step_error = sqrt(sum(step_error.^2,2)); % mean squared error for each timestep
                        isOutlier = (step_error>obj.outlierThreshold); % identify outliers
                        step_error(isOutlier) = obj.outlierThreshold; % prune outlier error
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
                        dpos_in = obj.wDp(~best_isOutlier,:);
                        dt_in = wDt(~best_isOutlier);

                        [obj.dx_est(1), obj.dv_est(1)] = linearLeastSquares(dt_in,dpos_in(:,1),obj.ransacPrior);
                        [obj.dx_est(2), obj.dv_est(2)] = linearLeastSquares(dt_in,dpos_in(:,2),obj.ransacPrior);
                        [obj.dx_est(3), obj.dv_est(3)] = linearLeastSquares(dt_in,dpos_in(:,3),obj.ransacPrior);
                    else
                        obj.dx_est = best_dp;
                        obj.dv_est = best_dv;
                    end
                    obj.last_error_update = time;
                end   
            end
            if obj.windowSize == 0 % first iteration
                dt = 0;
            else
                dt = time - obj.wt(obj.windowSize);
            end
            
            pred = QuadrotorState();
            pred.p = obj.x_pred;
            pred.v = obj.v_pred;
            pred.timestamp = time;
            
            est = QuadrotorState();
            est.p = obj.x_pred + obj.dx_est + dt*obj.dv_est;
            est.v = obj.v_pred + obj.dv_est;
            est.timestamp = time;
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
            obj.x_pred = obj.initialState.p;
            obj.v_pred = obj.initialState.v;
            obj.t_pred = 0;
            obj.dx_est = [0,0,0];
            obj.dv_est = [0,0,0];
            obj.last_error_update = 0;
            obj.wt = zeros(obj.maxWindowSize,1);
            obj.wDp = zeros(obj.maxWindowSize,3);
            obj.windowSize = 0;
        end
    end
    
    methods(Access = private)
        function obj = updatePrediction(obj,attitude,time)
            dt = time - obj.t_pred;
            obj.t_pred = time;
                      
            p_global = attitude(2) * cos(attitude(3)) + attitude(1)*sin(attitude(3));
            r_global = -attitude(2) * sin(attitude(3)) + attitude(1)*cos(attitude(3));
        
            obj.x_pred(1) = obj.x_pred(1) + obj.v_pred(1) * dt;
            obj.x_pred(2) = obj.x_pred(2) + obj.v_pred(2) * dt;
            
            obj.v_pred(1) = obj.v_pred(1) + (-obj.g*tan(p_global) - obj.k*obj.v_pred(1)) * dt;
            obj.v_pred(2) = obj.v_pred(2) + ( obj.g*tan(r_global) - obj.k*obj.v_pred(2)) * dt;

        end
        
        function wDt = updateWindows(obj, measurement, time)
            obj.wt = [time; obj.wt(1:end-1)];
            obj.wDp = [measurement.p - obj.x_pred; obj.wDp(1:end-1,:)];
            
            if (obj.windowSize < obj.maxWindowSize)
                obj.windowSize = obj.windowSize + 1;
            end
            while ((obj.wt(1)-obj.wt(obj.windowSize)) > obj.timeHorizon)
                obj.windowSize = obj.windowSize -1;
            end
            wDt = obj.wt - obj.wt(obj.windowSize);           
        end    
    end
end
