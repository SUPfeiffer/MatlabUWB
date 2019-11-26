function traj = trajSpline(p0,p1,v0,v1,time)
    N = length(time);

    a0 = p0;
    a1 = v0;
    a2 = 3*p1 - v1 - 2*a1 - 3*a0;
    a3 = v1 - 2*p1 + a1 + 2*a0;

    traj(1,:)=p0;
    for i=2:N
        t = time(i)/time(N);
        traj(i,:) = a3*t^3 + a2*t^2 + a1*t + a0;
    end

end