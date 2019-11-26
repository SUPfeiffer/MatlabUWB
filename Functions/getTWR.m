function twr_s = getTWR(anchors, pos, time)
    % Calculates a twr measurement from a random Anchors
    % @param anchors: array with anchor positions (one row per anchor) [x,y,z]
    % @param pos: current position [x,y,z]
    % @return twr_s.a: position anchor
    % @return twr_s.distance: distance to anchor
    
    id = randi(size(anchors,1));
    twr_s.a = anchors(id,:);
    twr_s.distance = norm(pos-twr_s.a);
    twr_s.mode = 'twr';
    twr_s.time = time;
end