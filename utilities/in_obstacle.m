%given a list of obstacles return if a point is in one of them
function state = in_obstacle(q, O)
    for i = 1:size(O, 1)
        if ((q(1) >= O(i, 1)) && (q(2) >= O(i, 2)) && ...
            (q(1) <= O(i, 1) + O(i, 3)) && (q(2) <= O(i, 2) + O(i, 4)))
            state = true;
            return
        end
    end
    state = false;
end

