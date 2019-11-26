function [b,m] = linearLeastSquares(x, y, P)
    % finds the optimal solution in the least square sense of y = mx + b
    % with a penalty matrix P = [pb,0 ; 0, pm] that penalizes large values
    % of b or m respectively to avoid overfitting.
    
    N = length(y);
    X = [ones(N,1), x];
    beta = inv(X'*X + P) * X' * y;
    %denom = N * sum(x.*x) - (sum(x))^2;
    %b = (sum(x.*x)*sum(y) - sum(x.*y)*sum(x))/denom;
    %m = (N * sum(x.*y) - sum(x)*sum(y))/denom;
    b = beta(1);
    m = beta(2);
end