classdef QuadrotorState
    %QuadrotorState: Basic state-variables for a quadrotor with timestamp
    %   No angular velocities atm
    
    properties
        p = [0,0,0]         % x, y, z
        v = [0,0,0]         % vx, vy, vz
        attitude = [0,0,0]  % roll, pitch, yaw
        omega = [0,0,0]     % roll, pitch, yaw - rates
        acc = [0,0,0]       % ax,ay,az
        timestamp = 0       % t at which state was computed
    end
    
    methods
        function obj = QuadrotorState()
            %QuadrotorState Construct an instance of this class
            
        end
        function r = plus(obj1, obj2)
            r = QuadrotorState;
            r.p = obj1.p + obj2.p;
            r.v = obj1.v + obj2.v;
            r.attitude = obj1.attitude + obj2.attitude;
            r.omega = obj1.omega + obj2.omega;
            r.timestamp = min([obj1.timestamp, obj2.timestamp]);
        end
        function r = minus(obj1, obj2)
            r = QuadrotorState;
            r.p = obj1.p - obj2.p;
            r.v = obj1.v - obj2.v;
            r.attitude = obj1.attitude - obj2.attitude;
            r.omega = obj1.omega - obj2.omega;
            r.timestamp = min([obj1.timestamp, obj2.timestamp]);
        end    
    end
end

