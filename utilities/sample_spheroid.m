% ellipsoidal sampling heuristic from RRT* Informed
function [q, plot] = sample_spheroid(map, c_max, plot)
    q = [Inf;Inf];
    if c_max < Inf
        while q(1) < map.bounds(1) || q(1) > map.bounds(2) || q(2) < map.bounds(3) || q(2) > map.bounds(4)
            c_min = norm(map.start - map.goal) - map.goal_r;
            xc = (map.start + map.goal) / 2;
            r1 = c_max /2;
            r2 = sqrt(c_max^2 - c_min^2) / 2;
            L = diag([r1 r2]);
            t = atan2(map.start(2) - map.goal(2), map.start(1) - map.goal(1));
            C = [cos(t) -sin(t); sin(t) cos(t)];
            q = sample_ball(0, 0, 1);
            q = (C * L * q' + xc')';
            plot = clear_sampler(plot);
            plot = ellipse(plot, r1, r2, t, xc(1), xc(2), 'k');
        end
    else
        q =[rand*map.bounds(2), rand*map.bounds(4)];
    end
end
