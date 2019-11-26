function traj = trajPoints(points, time)
    n_points = size(points,1);
    n_edges = n_points - 1;
    n_time = length(time);
    
    for i=1:n_edges
        length_edge(i) = norm(points(i+1,:)-points(i,:));
    end
    
    length_tot = sum(length_edge);
    
    idx_corner(1) = 1;
    traj(idx_corner(1),:) = points(1,:);
    for i=2:n_points
        frac = sum(length_edge(1:i-1))/length_tot;
        idx_corner(i) = round(frac*n_time);
        traj(idx_corner(i),:) = points(i,:);
    end
    
    for i=1:n_edges
        frags_on_edge = idx_corner(i+1)-idx_corner(i);
        frag_length = length_edge(i)/frags_on_edge;
        for j=1:frags_on_edge-1
            frag_idx = idx_corner(i)+j;
            traj(frag_idx,:) = traj(frag_idx-1,:) + frag_length * (points(i+1,:)-points(i,:))/length_edge(i);
        end
    end
end