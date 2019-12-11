function twr_s = getTWR(anchors, pos, time)
    % Calculates a twr measurement from a random Anchors
    % @param anchors: array with visible anchor positions (one row per anchor) [x,y,z]
    % @param pos: current position [x,y,z]
    % @return twr_s.anchor: position anchor
    % @return twr_s.distance: distance to anchor
    % @return twr_s.mode: 'twr'
    % @return twr_s.timestamp: time at which the measurement is generated
    
    id = randi(size(anchors,1));
    twr_s.anchor = anchors(id,:);
    twr_s.distance = norm(pos-twr_s.anchor);
    twr_s.mode = 'twr';
    twr_s.timestamp = time;
end