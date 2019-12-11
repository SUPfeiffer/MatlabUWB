function pos = tdoa2position_s(tdoa,prior)
    % Calculates a position estimate based on a SINGLE tdoa measurement from 
    % 2 Anchors
    % @param tdoa.a1,tdoa.a2: positions of the two anchors [x,y,z]
    % @param tdoa.dist_diff: the tdoa measurement
    % @param prior: prior knowledge of position
    % @return pos: the projection of the prior onto the tdoa measurement
    %               [x,y,z]
    
    a1 = tdoa.a1'; % turn into column vectors
    a2 = tdoa.a2';
    X = [prior'; 0];
    F = zeros(4,1);
    timeout_max_i = 20;
    exit_timeout = 0;
    while (true)
        d1 = norm(X(1:3)-a1);
        d2 = norm(X(1:3)-a2);

        gradS = (X(1:3)-a1)/d1 - (X(1:3)-a2)/d2;

        F(1:3) = X(1:3) + X(4)*gradS - prior';
        F(4) = d1 - d2 - tdoa.dist_diff;

        J(1:3,1:3) = eye(3) + X(4)*( (eye(3)/d1 - ((X(1:3)-a1)*(X(1:3)-a1)')/(d1^3)) ...
                                    -(eye(3)/d2 - ((X(1:3)-a2)*(X(1:3)-a2)')/(d2^3)) );
        J(1:3,4) = gradS;
        J(4,1:4) = [gradS; 0]';
        
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
            F_new(4) = d1 - d2 - tdoa.dist_diff;
            if (norm(F_new)<=norm(F))
                break;
            elseif (i>timeout_max_i)
                exit_timeout = 1;
                break;
            else
                i = i+1;
            end
        end
        X = X_new;
        if ((abs(delta)<0.001) == ones(4,1))
            break;
        elseif (exit_timeout)
            break;
        end
    end
    pos = X(1:3)';
end