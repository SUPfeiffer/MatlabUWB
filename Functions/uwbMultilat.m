function position = uwbMultilat(uwb, varargin)
%uwbMultilat: calculates position from uwb mesurements
%   uwb: array of uwb measurements, both TWRPacket and TDOAPacket types are
%           accepted
%   position: the optimal position in the least square sense
%
%   name-value parameters:
%   prior = [0,0,0]:    prior estimate of the position, not required if 
%                       uwb measurements are of type TWRPacket
%   timeout = Inf:      specifies the time of the oldest measurements that
%                       should still be used
%   nbest = Inf:        only consider the nbest best measurements


    % read name-value parameters
    p = inputParser();
    addParameter(p,'prior',[0,0,0]);
    addParameter(p,'timeout', 0);
    addParameter(p,'nbest', inf);
    addParameter(p,'forceZ', nan);
    parse(p,varargin{:});
    prior = p.Results.prior;
    timeout = p.Results.timeout;
    nbest = p.Results.nbest;
    forceZ = p.Results.forceZ;

    %% Pre-processing of measurements
    % remove outdated measurements
    if timeout
        for i=length(uwb):1
            if uwb(i).timestamp < timeout
                uwb(i) = [];
            end
        end
    end
    % only keep the nbest best measurements
    if ~isinf(nbest)
        for i=1:length(uwb)
            % Measure of the quality of a measurement could be changed, in
            % reality, one could for example keep track of differences between
            % measurement and prediction
            quality(i) = uwb(i).timestamp;
        end
        for i=1:nbest
            idx(i) = find(quality == max(quality),1);
            quality(idx(i)) = -1;
        end
        uwb = uwb(idx);
    end

    %% Calculate position from pre-processed measurements
    N = length(uwb);
    mode = uwb(1).rangingMode;
    if mode==rangingModes.twr
        % TWR measurements can be arranged in a set of linear equations
        % -> Analytical solution
        for i=1:N
        x(i) = uwb(i).anchorPosition(1);
        y(i) = uwb(i).anchorPosition(2);
        z(i) = uwb(i).anchorPosition(3);
        d(i) = uwb(i).distance;
        end
        
        if isnan(forceZ)
            A = [ones(N,1), -2*x', -2*y', -2*z'];
            b = d'.^2 -x'.^2 - y'.^2 - z'.^2;
            res = A\b;
            position = [res(2),res(3),res(4)];
        else
            % Force Z
            A = [ones(N,1), -2*x', -2*y'];
            b = d'.^2 - x'.^2 - y'.^2 - (z'-forceZ).^2;
            res = A\b;
            position = [res(2),res(3),forceZ];
            
            % "Suggest" Z (add as additional equation)
%             A = [ones(N,1), -2*x', -2*y', -2*z'];
%             A = [A; 0,0,0,1];
%             b = d'.^2 -x'.^2 - y'.^2 - z'.^2;
%             b = [b; forceZ];
%             res = A\b;
%             position = [res(2),res(3),res(4)];
            
        end
        
    elseif mode==rangingModes.tdoa
        % No analytical solution for tdoa 
        % -> numerical solution with Gauss-Newton
        position = prior;
        if ~isnan(forceZ)
            position(3) = forceZ;
        end
        while true
            for i=1:N
                dA = position - uwb(i).anchorPosition_A;
                dB = position - uwb(i).anchorPosition_B;
                S(i) = norm(dA) - norm(dB) - uwb(i).distance;
                
                if ~isnan(forceZ)
                    J(i,:) = dA(1:2)/norm(dA) - dB(1:2)/norm(dB);
                    J_valid = (rank(J)>= 2);
                else
                    J(i,:) = dA/norm(dA) - dB/norm(dB);
                    J_valid = (rank(J) >= 3);
                end
            end
            
            if ~J_valid
                break;
            end
            delta = J\S';
            if ~isnan(forceZ)
                delta(3) = 0;
            end
                
            position = position - delta';
            
            if norm(delta) < 0.001
                break;
            end    
        end
        
    end
end

