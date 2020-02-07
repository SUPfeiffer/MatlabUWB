classdef QuadrotorState
    %QuadrotorState: Basic state-variables for a quadrotor with timestamp
    %   No angular velocities atm
    
    properties
        p = [0,0,0]
        v = [0,0,0]
        roll = 0
        pitch = 0
        yaw = 0
        timestamp = 0
    end
    
    methods
        function obj = QuadrotorState()
            %QuadrotorState Construct an instance of this class
            
        end
        function r = plus(obj1, obj2)
            r = QuadrotorState;
            r.p = obj1.p + obj2.p;
            r.v = obj1.v + obj2.v;
            r.roll = obj1.roll + obj2.roll;
            r.pitch = obj1.pitch + obj2.pitch;
            r.yaw = obj1.yaw + obj2.yaw;
            r.timestamp = min([obj1.timestamp, obj2.timestamp]);
        end
        function r = minus(obj1, obj2)
            r = QuadrotorState;
            r.p = obj1.p - obj2.p;
            r.v = obj1.v - obj2.v;
            r.roll = obj1.roll - obj2.roll;
            r.pitch = obj1.pitch - obj2.pitch;
            r.yaw = obj1.yaw - obj2.yaw;
            r.timestamp = min([obj1.timestamp, obj2.timestamp]);
        end    
    end
end

