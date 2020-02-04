classdef TWRPacket < handle
    %TWRPacket: Holds information that is extracted from TWR messages on a
    %           real drone
    
    properties
        rangingMode (1, 1) rangingModes = rangingModes.twr
        anchorID (1, 1) int32 {mustBeNonnegative, mustBeFinite}
        anchorPosition (1, 3) double {mustBeFinite}
        distance (1, 1) double {mustBeFinite}
        timestamp (1, 1) double {mustBeNonnegative} = 0
    end
    
    methods
        function obj = TWRPacket(anchor, distance, timestamp)
            %UWBPacket Construct an instance of this class
            if nargin > 0
                obj.anchorID = anchor.id;
                obj.anchorPosition = anchor.position;
                obj.distance = distance;
                obj.timestamp = timestamp;
            end
        end
    end
end

