function tdoa_s = getTDOA(anchors, pos, time)
    % Calculates a tdoa measurement from 2 random Anchors
    % @param anchors: array with anchor positions (one row per anchor) [x,y,z]
    % @param pos: current position [x,y,z]
    % @return tdoa_s.a1: position anchor 1
    % @return tdoa_s.a2: position anchor 2
    % @return tdoa_s.dist_diff: distance difference
    
    id1 = randi(size(anchors,1));
    id2 = randi(size(anchors,1));
    while (id1 == id2)
        id2 = randi(size(anchors,1));
    end
    tdoa_s.a1 = anchors(id1,:);
    tdoa_s.a2 = anchors(id2,:);
    d1 = norm(pos-tdoa_s.a1);
    d2 = norm(pos-tdoa_s.a2);
    tdoa_s.dist_diff = d1-d2;
    tdoa_s.mode = 'tdoa';
    tdoa_s.timestamp = time;
end