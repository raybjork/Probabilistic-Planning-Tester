function [J, k, t] = rrt_star_informed(varargin)   
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
    r = 8;                         % radius of ball to check
    refresh_rate = 50;              % update after this many test points
    
    % initialize an array of configurations that are sufficiently close to
    %   the goal, store an int for the size of the array
    goal_qs = k;   
    num_goal_qs = 1;

    for i = goal_qs + 1 : N + 1        
        % refresh plot and calculate solution
        if mod(i, refresh_rate) == 0
            t_i = toc;
            plot.refresh();
            [c, ~] = best_path(A, qs, goal_qs, 1, 0);
            fprintf("Cost after %d samples: %.1f\n", i-1, c)
               
            % update test data
            J = [J; c]; %#ok<AGROW>
            k = [k; i-1]; %#ok<AGROW>
            t = [t; t(end) + t_i]; %#ok<AGROW>
            tic;
        end
        
        % generate a random sample
        nodes = diag(A);
        cost = min(nodes(goal_qs));
        [qs(i, :), plot] = sample_spheroid(map, cost, plot);

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
                    break;
                end
            end  
        end
    end
end

