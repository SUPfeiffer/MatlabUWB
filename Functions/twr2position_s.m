function pos = twr2position_s(twr,prior)
    % Calculates a position estimate based on a SINGLE twr measurement from 
    % one Anchors
    % @param twr.a: positions of the two anchors [x,y,z]
    % @param twr.distance: the tdoa measurement
    % @param prior: prior knowledge of position
    % @return pos: the projection of the prior onto the tdoa measurement
    %               [x,y,z]
    
    pos = twr.a + twr.distance * (prior-twr.a)/norm(prior-twr.a);
   
end