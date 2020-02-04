function tdoa_s = getTDOA(anchors, pos, time)
    % Calculates a tdoa measurement from 2 random Anchors
    % @param anchors: array with anchor objects
    % @param pos: current position [x,y,z]
    % @return tdoa_s: TDOAPacket Object
    
    id1 = randi(size(anchors,1));
    id2 = randi(size(anchors,1));
    while (id1 == id2)
        id2 = randi(size(anchors,1));
    end
    anchor_A = anchors(id1);
    anchor_B = anchors(id2);
    dA = norm(pos-anchor_A.position);
    dB = norm(pos-anchor_B.position);
    dist_diff = dA-dB;
    
    tdoa_s = TDOAPacket(anchor_A,anchor_B,dist_diff,time);
end