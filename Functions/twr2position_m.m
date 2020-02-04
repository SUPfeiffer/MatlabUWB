function pos = twr2position_m(twr)
%twr2position_m Calculate position by triangulation of N twr measurements
%   Detailed explanation goes here

% NOTE: A is singular for 4 anchors in a plane
N = length(twr);
for i=1:N
    x(i) = twr(i).anchorPosition(1);
    y(i) = twr(i).anchorPosition(2);
    z(i) = twr(i).anchorPosition(3);
    d(i) = twr(i).distance;
end
    
A = [ones(N,1), -2*x', -2*y', -2*z'];
b = d'.^2 -x'.^2 - y'.^2 - z'.^2;

res = A\b;
pos = [res(2),res(3),res(4)];
end

