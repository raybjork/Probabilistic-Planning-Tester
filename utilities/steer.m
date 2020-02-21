% Function to steer RRT in the direction of a newly sampled node
function new = steer(near, target, step)
    dist = norm(near - target);
    if dist <= step
        new = target;
    else
        new = near + ((target - near) * step) / dist;
    end
end