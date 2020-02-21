% This function is used to simulate a single algorithm on a single map type
function [J, K, T] = tester(planner, seeder, map_num, num_tests)
    sum = 0;
    J = cell(1, num_tests);
    K = cell(1, num_tests);
    T = cell(1, num_tests);
    for i = 1:num_tests
        [j, k, t] = planner(select_map(map_num), seeder);
        J(i) = {j};
        K(i) = {k};
        T(i) = {t};       
    end
    fprintf("Average cost for %d tests: %.1f\n", num_tests, sum / num_tests)

end