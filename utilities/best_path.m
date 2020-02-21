% this function navigates an adjacency matrix  and vertex list to return
% the shortest path between two points of the graph
function [cost, plot, path] = best_path(A, qs, goal_qs, origin, plot)
    if ~isempty(goal_qs)
        [path, cost] = shortest_path(A, goal_qs, origin);
        if ~isa(plot, 'double')
            plot = clear_path(plot);
            plot = plot_edges(plot, qs(path(1:end-1,:), :), ...
                                    qs(path(2:end,:), :));
        end
    else
        cost = Inf;
    end
end

% recursive function that actually does the shortest path computation
function [path, cost] = shortest_path(A, family, origin)
    % recursive case
    nodes = diag(A);
    [x, i] = min(nodes(family));
    if (family(i) ~= origin) 
        [path, y] = shortest_path(A, find(A(family(i),:)), origin);
        path = [family(i); path];
        cost = max(x, y);
    % base case
    else 
        cost = x;
        path = i;
    end
end