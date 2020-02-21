function [J, k, t] = rrt_star_smart(varargin)
    % initialize planner with a seed
    [A, qs, cost, plot, map] = seed_planner(varargin);
    
    % initialize test parameters
    J = cost;
    t = 0;
    k = max(find(diag(A))); %#ok<*MXFND>
    tic;
    
    % declaration of constants
    N = size(qs, 1) - 2;            % max number of samples taken
    step = 4;                       % max step size given by steer
    r = 8;                          % radius of ball to check
    b = 2;                          % sample bias ratio 1:b
    beacon_r = 10;                  % beacon radius to search
    refresh_rate = 50;              % refresh plot after this many samples

    % initialize an array of configurations that are sufficiently close to
    %   the goal, store an int for the size of the array
    goal_qs = max(find(diag(A)));   %#ok<*MXFND>
    num_goal_qs = 1;
    
    % find the optimal path from the seeded trajectory
    [~, ~, path] = best_path(A, qs, goal_qs, 1, 0);
    [A, plot, beacons, cost] = optimize_path(A, qs, path, map.obstacles, plot);
    plot = clear_path(plot);
    plot = plot_edges(plot, qs(beacons(1:end-1,:), :), qs(beacons(2:end,:), :), -1);
    
    % continue sampling from where you left off in configuration matrix
    n = goal_qs;
    for i = n + 1 : N + 1        
        % refresh plot and calculate solution
        if mod(i, refresh_rate) == 0
            t_i = toc;
            plot.refresh();
            fprintf("Cost after %d samples: %.1f\n", i-1, cost)
               
            % update test data
            J = [J; cost]; %#ok<AGROW>
            k = [k; i-1]; %#ok<AGROW>
            t = [t; t(end) + t_i]; %#ok<AGROW>
            tic;
        end
        
        % generate a random sample
        if mod(i - n, b) == 0
            x = randi(length(beacons) - 1, 1);
            qs(i, :) = sample_ball(qs(beacons(x), 1), ...
                                   qs(beacons(x), 2), beacon_r);
        else
            qs(i, :) = [rand*map.bounds(2), rand*map.bounds(4)];
        end

        % steer random sample towards closest node
        closest = closest_node(A, qs, i);
        qs(i, :) = steer(qs(closest, :), qs(i, :), step);        
        
        % if the sample is not inside of an obstacle...
        if ~in_obstacle(qs(i, :), map.obstacles)
            
            % find its near neighbors and iterate through them
            neighbors = k_nearest(A, qs, i, r, 1);
            for j = 1 : size(neighbors, 1)
                
                % check the trajectory to the near neighbor for obstacles
                if check_trajectory(qs(neighbors(j, 1), :), qs(i, :), map.obstacles)
                    
                    % add it to the plot
                    [A, plot] = insert_edge(A, qs, neighbors(j, 1), i, plot);
                    
                    % rewire its neighbors to it if more efficient
                    [A, plot] = rewire(A, qs, neighbors, i, plot, map.obstacles);
                    
                    % if it close to the goal, add it to that list
                    if norm(qs(i, :) - map.goal) < map.goal_r
                        num_goal_qs = num_goal_qs + 1;
                        goal_qs(num_goal_qs) = i;
                    end

                    % check for a better trajectory
                    [~, ~, path] = best_path(A, qs, goal_qs, 1, 0);
                    [A, plot, path, new_cost] = optimize_path(A, qs, path, map.obstacles, plot);
                    if new_cost < cost
                        
                        % update cost and path
                        cost = new_cost;
                        beacons = path;
                        plot = clear_path(plot);
                        plot = plot_edges(plot, qs(path(1:end-1,:), :), qs(path(2:end,:), :), -1);
                    end
                    break;
                end
            end  
        end
    end
end