function pos = twr2position_m_wz(twr1,twr2, twr3, twr4, z_ext)
%twr2position_m Calculate position by triangulation of 4 twr measurements
%   Detailed explanation goes here

twr = [twr1, twr2, twr3, twr4];

for i=1:4
    x(i) = twr(i).anchor(1);
    y(i) = twr(i).anchor(2);
    z(i) = twr(i).anchor(3);
    d(i) = twr(i).distance;
end
    
%A = [ones(4,1), -2*x', -2*y', -2*z';
%            0 ,    0 ,    0 ,    1 ];
        
%tmp = d.^2 - x.^2 - y.^2 - z.^2;
%b = [tmp, z_ext];

A = [ones(4,1), -2*x', -2*y'];
b = d.^2 - x.^2 - y.^2 - (z-z_ext).^2;

res = A\b';
%pos = [res(2),res(3),res(4)];
pos = [res(2),res(3),z_ext];
end

