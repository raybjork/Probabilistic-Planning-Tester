function  [A, qs, cost, plot] = rrt(map, ~)
    %% setup
    % declaration of constants
    N = 3000;                       % max number of samples taken
    step = 10;                       % max step size given by steer
    refresh_rate = 10;              % update after this many test points

    % set up system plotter
    plot = Plotter(N);
    plot.setup(map);

    % initialize array of configurations (samples will add to it)
    qs = zeros(N + 2, 2);
    qs(1, :) = map.start;
    qs(end, :) = map.goal;

    % initialize (N+2 x N+2) adjacency matrices representing search trees where
    %   node N + 2 represents the goal configuration
    %   node 1 represents the start configuration
    %   and index i,i represents configuration i is in the tree
    A = zeros(N + 2);
    A(1,1) = 1;
    
    % loop through until max number of samples have been taken
    for i = 2: N + 1
        % exit if it takes too long
        if i > N/2
            cost = Inf;
            return
        end
        
        % refresh plot and calculate solution
        if mod(i, refresh_rate) == 0
            plot.refresh();
        end
        
        % generate a random sample
        qs(i, :) = [rand*map.bounds(2), rand*map.bounds(4)];

        % steer random sample towards closest node
        closest = closest_node(A, qs, i);
        qs(i, :) = steer(qs(closest, :), qs(i, :), step);        
        
        % check the trajectory to the near neighbor for obstacles
        if check_trajectory(qs(closest, :), qs(i, :), map.obstacles)
                    
            % add it to the plot
            [A, plot] = insert_edge(A, qs, closest, i, plot);

            % if it close to the goal, add it to that list
            if norm(qs(i, :) - map.goal) < map.goal_r
                [cost, plot] = best_path(A, qs, i, 1, plot);
                fprintf("Cost after %d samples: %.1f\n", i-1, cost)
                return;
            end
        end
    end
end