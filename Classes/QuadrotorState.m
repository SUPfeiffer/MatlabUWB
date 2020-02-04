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
    end
end

