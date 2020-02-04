function twr_s = getTWR(anchors, pos, time)
    % Calculates a twr measurement from a random Anchors
    % @param anchors: array with anchor objects
    % @param pos: current position [x,y,z]
    % @return TWRPacket object
    
    id = randi(size(anchors,1));
    anchor = anchors(id);
    dist = norm(pos-anchor.position);
    
    twr_s = TWRPacket(anchor, dist, time);
end