classdef TDOAPacket < handle
    %TDOAPacket: Holds information that is extracted from TDOA messages on a
    %           real drone
    
    properties
        rangingMode (1, 1) rangingModes = rangingModes.tdoa
        anchorID_A %(1, 1) int32 {mustBeNonnegative, mustBeFinite}
        anchorPosition_A %(1, 3) double {mustBeFinite}
        anchorID_B %(1, 1) int32 {mustBeNonnegative, mustBeFinite}
        anchorPosition_B %(1, 3) double {mustBeFinite}
        distance %(1, 1) double {mustBeFinite} = 0 
        timestamp %(1, 1) double {mustBeNonnegative} = 0
    end
    
    methods
        function obj = TDOAPacket(anchor_A, anchor_B, distance, timestamp)
            %TDOAPacket Construct an instance of this class
            if nargin > 0
                obj.anchorID_A = anchor_A.id;
                obj.anchorPosition_A = anchor_A.position;
                obj.anchorID_B = anchor_B.id;
                obj.anchorPosition_B = anchor_B.position;
                obj.distance = distance;
                obj.timestamp = timestamp;
            end
        end
        function switchAnchors(obj)
            % swap anchor A & B and adjust distance accordingly
            tmpID = obj.anchorID_A;
            tmpPosition = obj.anchorPosition_A;
            obj.anchorID_A = obj.anchorID_B;
            obj.anchorPosition_A = obj.anchorPosition_B;
            obj.anchorID_B = tmpID;
            obj.anchorPosition_B = tmpPosition;
            obj.distance = - obj.distance;
        end
    end
end

