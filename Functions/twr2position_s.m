function pos = twr2position_s(twr,prior)
    % Calculates a position estimate based on a SINGLE twr measurement from 
    % one Anchors
    % @param twr.anchorPosition: position of the anchor [x,y,z]
    % @param twr.distance: the twr measurement
    % @param prior: prior knowledge of position
    % @return pos: the projection of the prior onto the tdoa measurement
    %               [x,y,z]
    
    pos = twr.anchorPosition + twr.distance * (prior-twr.anchorPosition)/norm(prior-twr.anchorPosition);
   
end