function position = uwbProject(uwb,prior,varargin)
%uwbProject: calculates the projection of the prior position estimate onto
%   the uwb measurement.
%   
%   uwb: uwb measurement as TWRPacket or TDOAPacket
%   prior: prior estimate of the position

% read name-value parameters
    p = inputParser();
    addParameter(p,'forceZ', nan);
    parse(p,varargin{:});
    forceZ = p.Results.forceZ;
    
    % calculation
    mode = uwb.rangingMode;
    if mode == rangingModes.twr
        if isnan(forceZ)
            dir = (prior-uwb.anchorPosition)/norm(prior-uwb.anchorPosition);
            position = uwb.anchorPosition + uwb.distance * dir;
        else
            prior2D = prior(1:2);
            anchor2D = uwb.anchorPosition(1:2);
            distance2D = sqrt(uwb.distance^2 -(forceZ-uwb.anchorPosition(3))^2);
            
            dir = (prior2D-anchor2D)/norm(prior2D-anchor2D);
            position2D = anchor2D + distance2D*dir;
            position = [position2D, forceZ];
        end
    
    elseif mode == rangingModes.tdoa
        % turn anchor positions into column vectors
        a1 = uwb.anchorPosition_A'; 
        a2 = uwb.anchorPosition_B';
        % variables for timeout if no convergence
        timeout_max_i = 5; 
        exit_timeout = 0;     
        
        if isnan(forceZ)
            % Initialize optimization problem
            X = [prior'; 0];    % Candidate Solution (position on tdoa measurement)
            F = zeros(4,1);     % Objective function
            while (true)

                d1 = norm(X(1:3)-a1);
                d2 = norm(X(1:3)-a2);

                % normal vector to the tdoa measurement (hyperboloid) at candidate solution
                gradS = (X(1:3)-a1)/d1 - (X(1:3)-a2)/d2;

                % Objective function
                F(1:3) = X(1:3) + X(4)*gradS - prior';   
                F(4) = d1 - d2 - uwb.distance;

                % Jacobian of the objective function
                J(1:3,1:3) = eye(3) + X(4)*( (eye(3)/d1 - ((X(1:3)-a1)*(X(1:3)-a1)')/(d1^3)) ...
                                            -(eye(3)/d2 - ((X(1:3)-a2)*(X(1:3)-a2)')/(d2^3)) );
                J(1:3,4) = gradS;
                J(4,1:4) = [gradS; 0]';

                % update for the candidate solution
                delta = J\F;

                % backtracking line search to ensure convergence if prior is far
                % from root
                i = 0;
                while(true)
                    X_new = X - (0.5^i) * delta;

                    d1 = norm(X_new(1:3)-a1);
                    d2 = norm(X_new(1:3)-a2);
                    gradS = (X_new(1:3)-a1)/d1 - (X_new(1:3)-a2)/d2;
                    F_new(1:3) = X_new(1:3) + X_new(4)*gradS - prior';
                    F_new(4) = d1 - d2 - uwb.distance;
                    if (norm(F_new)<=norm(F))
                        break;
                    elseif (i>timeout_max_i)
                        % timeout if no convergence for too long
                        exit_timeout = 1;
                        break;
                    else
                        i = i+1;
                    end
                end
                X = X_new;
                if norm(delta)<0.001
                    break;
                elseif (exit_timeout)
                    % forced exit by timeout
                    break;
                end
            end
            position = X(1:3)';
        else
            % z forced, doesn't work properly yet!
            prior = prior([1,2]);
            X = [prior'; 0];    % Candidate Solution (position on tdoa measurement)
            F = zeros(3,1);     % Objective function
            while (true)
                d1 = norm([X(1:2);forceZ]-a1);
                d2 = norm([X(1:2);forceZ]-a2);

                % normal vector to the tdoa measurement (hyperboloid) at candidate solution
                gradS = (X(1:2)-a1(1:2))/d1 - (X(1:2)-a2(1:2))/d2;

                % Objective function
                F(1:2) = X(1:2) + X(3)*gradS - prior(1:2)';   
                F(3) = d1 - d2 - uwb.distance;

                % Jacobian of the objective function
                J(1,1) = 1 + X(3)*( (1/d1-(X(1)-a1(1))/d1^3) - (1/d2 - (X(1)-a2(1))/d2^3));
                J(1,2) = 0 + X(3)*( (X(1)-a1(1))*(X(2)-a1(2))/d1^3 - (X(1)-a2(1))*(X(2)-a2(2))/d2^3);
                
                J(2,1) = 0 + X(3)*( (X(1)-a1(1))*(X(2)-a1(2))/d1^3 - (X(1)-a2(1))*(X(2)-a2(2))/d2^3);
                J(2,2) = 1 + X(3)*( (1/d1-(X(2)-a1(2))/d1^3) - (1/d2 - (X(2)-a2(2))/d2^3));
                
%                J(1:2,1:2) = eye(2) + X(3)*( (eye(2)/d1 - ((X(1:2)-a1(1:2))*(X(1:2)-a1(1:2))')/(d1^3)) ...
%                                            -(eye(2)/d2 - ((X(1:2)-a2(1:2))*(X(1:2)-a2(1:2))')/(d2^3)) );
                J(1:2,3) = gradS;
                J(3,1:3) = [gradS; 0]';

                % update for the candidate solution
                delta = J\F;

                % backtracking line search to ensure convergence if prior is far
                % from root
                i = 0;
                while(true)
                    X_new = X - (0.5^i) * delta;

                    d1 = norm([X_new(1:2);forceZ]-a1);
                    d2 = norm([X_new(1:2);forceZ]-a2);
                    gradS = (X_new(1:2)-a1(1:2))/d1 - (X_new(1:2)-a2(1:2))/d2;
                    F_new(1:2) = X_new(1:2) + X_new(3)*gradS - prior(1:2)';
                    F_new(3) = d1 - d2 - uwb.distance;
                    if (norm(F_new)<=norm(F))
                        break;
                    elseif (i>timeout_max_i)
                        % timeout if no convergence for too long
                        exit_timeout = 1;
                        break;
                    else
                        i = i+1;
                    end
                end
                X = X_new;
                if norm(delta)<0.001 || norm(F)<0.001
                    break;
                elseif (exit_timeout)
                    % forced exit by timeout
                    break;
                end
            end
            position = [X(1:2);forceZ]';
        end
    end
end

