% This function takes in a seeder function a number of tests to run and a
% map number and simulates for 4 RRT* algorithms
function [J, K, T] = rrt_comparer(seeder, map_num, num_tests)
    planners = {@rrt_star, @rrt_star_informed, @rrt_star_smart, @rrt_star_new};

    J = cell(length(planners), num_tests);
    K = cell(length(planners), num_tests);
    T = cell(length(planners), num_tests);
    for a = 1:num_tests
        map = select_map(map_num);
        [A, qs, cost, plot] = seeder(map, 1);
        while cost == Inf
            [A, qs, cost, plot] = seeder(map, 1);
        end
        for i = 1:length(planners)
            fprintf("Test #%d: Planner#%d\n", a, i)
            planner = planners{i};
            [j, k, t] = planner(map, seeder, A, qs, cost, plot);
            J(i, a) = {j};
            K(i, a) = {k};
            T(i, a) = {t};       
            save map72 J K T
        end
    end
    %fprintf("Average cost for %d tests: %.1f\n", num_tests, sum / num_tests)

end
