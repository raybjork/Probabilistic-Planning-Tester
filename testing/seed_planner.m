function [A, qs, cost, plot, map, video] = seed_planner(arg)
    map = arg{1};
    seeder = arg{2};
    if length(arg) == 2
        % initialize by finding valid RRT path
        [A, qs, cost, plot] = seeder(map, true);
    else
        A = arg{3};
        qs = arg{4};
        cost = arg{5};
        plot = arg{6};
    end
end

