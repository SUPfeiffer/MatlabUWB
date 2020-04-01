classdef adaptiveUFIR < matlab.System
    % adaptive UFIR 
    %
    % adaptive UFIR state estimator based on
    % "Robust inertial navigation system / ultra wide band integrated 
    % indoor quadrotor localization employing adaptive interacting 
    % multiple model-unbiased finite impulse response / Kalman filter 
    % estimator", Xu et al. 2020


    % Public, tunable properties
    properties
        N_min = 7;
        N_max = 40;
        dt
    end

    properties(DiscreteState)
        x   % [x0;x1;...;x_k], xi = [dx, dvx, dy, dvy, dz, dvz]
        P
        N_opt
        prediction
        wDt
        wy   % [dx_m dy_m dz_m]
        t
        k
        t_pred
    end

    % Pre-computed constants
    properties(Access = private)
        initialState = QuadrotorState();
        % Model
        g = 9.81;   % gravitational constant
        k_aero = 0.35;   % drag coefficient
        M = 6;
        A
        C = [1, 0, 0, 0, 0, 0;
             0, 0, 1, 0, 0, 0;
             0, 0, 0, 0, 1, 0];
        G_init
        Q = 0.1 * eye(6);
        R = 0.1 * eye(3);
        
    end

    methods
        function obj = adaptiveUFIR(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:})
        end
    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.A = [1,obj.dt, 0, 0, 0, 0;
                    0, 1, 0, 0, 0, 0;
                    0, 0, 1,obj.dt, 0, 0;
                    0, 0, 0, 1, 0, 0;
                    0, 0, 0, 0, 1,obj.dt;
                    0, 0, 0, 0, 0, 1];
            H = [obj.C * inv(obj.A^5);
                 obj.C * inv(obj.A^4);
                 obj.C * inv(obj.A^3);
                 obj.C * inv(obj.A^2);
                 obj.C * inv(obj.A);
                 obj.C];
            obj.G_init = inv(H'*H);
        end

        function [pred, est] = stepImpl(obj, measurement, time)
            % Prediction
            obj.k = obj.k +1;
            obj.updatePrediction(measurement.attitude, time);
            obj.prediction(obj.k,5) = measurement.p(3);
            obj.addMeasurement(measurement, time);

            for N = obj.N_min:obj.N_max
                if obj.k < N-1
                    % initialization before full window is available (KF)
                    % prior
                    x_tmp = (obj.A * obj.x(obj.k-1,:)')';
                    obj.P(:,:,obj.k) = obj.A * obj.P(:,:,obj.k-1) * obj.A' + obj.Q;
                    % gain
                    K = obj.P(:,:,obj.k)*obj.C'*inv(obj.C*obj.P(:,:,obj.k)*obj.C' + obj.R);
                    % posterior
                    x_tmp = x_tmp + (K*(obj.wy(obj.k)'-obj.C*x_tmp'))';
                    obj.P(:,:,obj.k) = (eye(6)-K*obj.C)*obj.P(:,:,obj.k);
                    Dt(N) = 0;
                    xN(N,:) = x_tmp;

                else
                    m = obj.k - N + 1;
                    s = m + obj.M - 1;
                    x_tmp = obj.x(s,:);
                    G = obj.G_init;
                    for i=m+obj.M:obj.k
                        x_tmp = (obj.A * x_tmp')';
                        %obj.P(:,:,i) = A_i * obj.P(:,:,i-1) * A_i' + obj.Q;
                        yi = (obj.C*x_tmp')';
                        Dt(N) = (obj.wy(i,:)-yi)*inv(obj.R)*(obj.wy(i,:)-yi)';

                        G = inv(obj.C'*obj.C + inv(obj.A*G*obj.A'));
                        K = G*obj.C';
                        x_tmp = x_tmp + (K*(obj.wy(i,:)'-obj.C*x_tmp'))';
                        %obj.P(:,:,i) = ...
                    end
                    xN(N,:) = x_tmp;
                end                
            end
            Dt_short = Dt(obj.N_min:obj.N_max);
            obj.N_opt(obj.k) = find(Dt_short == min(Dt_short),1) + obj.N_min-1;
            obj.x(obj.k,:) = xN(obj.N_opt(obj.k),:);
            
            pred = QuadrotorState();
            pred.p = obj.prediction(obj.k,[1,3,5]);
            pred.v = obj.prediction(obj.k,[2,4,6]);
            pred.timestamp = time;
            real = obj.prediction(obj.k,:) - obj.x(obj.k,:);
            
            est = QuadrotorState();
            est.p = [real(1), real(3), real(5)];
            est.v = [real(2), real(4), real(6)];
            est.attitude = measurement.attitude;
            est.timestamp = time;
                
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
            obj.k = 1;
            obj.t = 0;
            obj.x = zeros(1,6);
            iX = obj.initialState.p; iV = obj.initialState.v;
            obj.prediction = [iX(1), iV(1), iX(2), iV(2), iX(3), iV(3)];
            obj.t_pred = 0;
            obj.wDt = 0;
            obj.wy = [0,0,0];
            obj.P = eye(6);
            obj.N_opt = 0;

        end
    end
    
    methods(Access = private)
        function obj = updatePrediction(obj,attitude,time)
            az = 0;
            obj.t_pred = time;
            
            p_global = attitude(2) * cos(attitude(3)) + attitude(1)*sin(attitude(3));
            r_global =-attitude(2) * sin(attitude(3)) + attitude(1)*cos(attitude(3));
            
            A_pred = obj.A; 
            A_pred(2,2) = 1-obj.k_aero*obj.dt; A_pred(4,4) = 1-obj.k_aero*obj.dt;
            b_pred = [0; -obj.g*tan(p_global); 0; obj.g*tan(r_global); 0; az] * obj.dt;
            obj.prediction(obj.k,:) = (A_pred * obj.prediction(obj.k-1,:)' + b_pred)';
        end
        
        function obj = addMeasurement(obj, meas, time)
            obj.wDt(obj.k) = time - obj.t;
            obj.t = time;
            obj.wy(obj.k,:) = (obj.C*obj.prediction(obj.k,:)')' - meas.p;
        end    
    end
end
