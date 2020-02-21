% rewire function for RRT*
function [A, plot] = rewire(A, configurations, neighbors, i, plot, O)
    for j = 1 : size(neighbors, 1)
        if (neighbors(j, 2) + A(i,i) < 2 * A(neighbors(j,1),neighbors(j,1))) && ...
            check_trajectory(configurations(neighbors(j, 1), :), configurations(i, :), O);
            plot = delete_edges(plot, neighbors(j, 1));
            A(neighbors(j, 1), i) = 0;
            A(i, neighbors(j, 1)) = 0;
            [A, plot] = insert_edge(A, configurations, i, neighbors(j, 1), plot);
        end
    end
end