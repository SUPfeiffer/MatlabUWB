classdef Anchor
    %Anchor: Holds information on an UWB Anchor    
    properties
        id
        position
    end
    
    methods
        function obj = Anchor(id,position)
            obj.id = id;
            obj.position = position;
        end
    end
end

