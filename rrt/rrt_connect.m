function [A, qs, cost, plot] = rrt_connect(map)
%% RRT 
% Find the shortest path from start to goal using a probablistic roadmap
%
% INPUTS:
%   map     - the map object to plan in
%   start   - vector of the starting configuration
%   goal:   - vector of the goal configuration
%
% OUTPUT: a mx6 matrix, where each row consists of the configuration of the
%   Lynx at a point on the path. The first row is start and the last row is
%   goal. If no path is found, PATH is a 0x6 matrix. 
% 
% AUTHOR
%   Ray Bjorkman - raybjork@seas.upenn.edu

    %% setup
    % declaration of constants
    N = 1000;                   % max number of samples taken
    refresh_rate = 10;              % update after this many test points

    % set up system plotter
    plot = Plotter(N);
    plot.setup(map);

    % initialize array of configurations (samples will add to it)
    qs = zeros(N + 2, 2);
    qs(1, :) = map.start;
    qs(N + 2, :) = map.goal;

    % initialize (N+2 x N+2) adjacency matrices representing search trees
    %   node N + 2 represents the goal configuration
    %   node 1 represents the start configuration
    %   and index i,i represents configuration i is in the tree
    start_tree = zeros(N + 2, N + 2);
    start_tree(1,1) = 1;
    goal_tree = zeros(N + 2, N + 2);
    goal_tree(end, end) = 1;

    %% randomly sample space and add it to the tree
    for i = 2: N + 1
        % refresh plot
        if mod(i, refresh_rate) == 0
            plot.refresh();
        end
            
        % generate a random sample
        qs(i, :) = [rand*map.bounds(2), rand*map.bounds(4)];

        % check the trajectory of the sampled point to its closest node to  
        % see if its valid, add edges if it is, plot
        [start_tree, start_valid] = test_point(map.obstacles, plot, ...
            start_tree, qs, i);
        [goal_tree, goal_valid] = test_point(map.obstacles, plot, ...
            goal_tree, qs, i);

        % if it is in both trees stop the algorithm and return path
        % from the start to goal of the composed adjacency matrices
        if start_valid && goal_valid
            A = start_tree + goal_tree;
            [path, cost] = shortestpath(graph(A), 1, N+2);
            if length(path) > 2
                plot_edges(plot, qs(path(1:end-1), :), qs(path(2:end), :));
            end
            fprintf("Cost: %f\n",cost)
            return
        end
    end
end

function [A, valid] = test_point(O, plot, A, configurations, index) 
    % find the closest node to each tree
    closest = closest_node(A, configurations, index);
    
    % check the trajectory to its closest node to see if its valid
    [valid, collisions] = check_trajectory(configurations(closest, :), ...
        configurations(index, :), O);
    
    % if it has a valid path to the start tree add an edge between itself
    % its closest node, add the node to the tree, and enforce symmetry
    if isempty(collisions)
        A = insert_edge(A, configurations, closest, index, plot);
    else
        %plot.collisions(collisions);
    end
end

