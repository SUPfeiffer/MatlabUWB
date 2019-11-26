function inRange = anchorsInRange(anchors,pos,range)
    % returns the subset of anchors which is within a given range
    j = 1;
    for i=1:size(anchors,1)
        if (norm(anchors(i,:)-pos)<range) 
            inRange(j,:) = anchors(i,:);
            j=j+1;
        end
    end

end