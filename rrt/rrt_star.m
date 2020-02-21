function [A, qs, cost, plot] = rrt_star(varargin)    
    % declaraion of constants
    step = 4;                       % max step size given by steer
    r = 8;                         % radius of ball to check
    refresh_rate = 50;              % update after this many test points

    
    % initialize planner with a seed
    if isa(varargin{2}, 'function_handle') || nargin > 2
        [A, qs, cost, plot, map] = seed_planner(varargin);
        pass = false;

        % initialize test parameters
        J = cost;
        t = 0;
        k = max(find(diag(A))); %#ok<*MXFND>
        tic;

        % declaration of constants
        N = size(qs, 1) - 2;            % max number of samples taken
        step = 5;                       % max step size given by steer
        r = 10;                         % radius of ball to check
        refresh_rate = 50;              % update after this many test points

        % initialize an array of configurations that are sufficiently close to
        %   the goal, store an int for the size of the array
        goal_qs = k;   
        num_goal_qs = 1;
        
        % index to start at
        start = goal_qs + 1;
    
    % use planner as a seed
    else
        % unpack arguments
        map = varargin{1};
        pass = varargin{2};
        
        % initialize test parameters
        J = Inf;
        t = 0;
        k = 2; %#ok<*MXFND>
        tic;
        
        % declaration of constants
        N = 5000;                       % max number of samples taken
        
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
        
        % initialize an array of configurations that are sufficiently close to
        %   the goal, store an int for the size of the array
        goal_qs = [];
        num_goal_qs = 0;
        
        % index to start at
        start = k;
    end
    
    for i = start: N + 1
        % refresh plot and calculate solution
        if mod(i, refresh_rate) == 0
            t_i = toc;
            plot.refresh();
            [cost, ~] = best_path(A, qs, goal_qs, 1, plot);
            fprintf("Cost after %d samples: %.1f\n", i-1, cost)
               
            % update test data
            J = [J; cost]; %#ok<AGROW>
            k = [k; i-1]; %#ok<AGROW>
            t = [t; t(end) + t_i]; %#ok<AGROW>
            tic;
        end
        
        % generate a random sample
        qs(i, :) = [rand*map.bounds(2), rand*map.bounds(4)];

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
                        
                        % pass results as a seed if true
                        if pass
                            plot.refresh();
                            [cost, plot] = best_path(A, qs, goal_qs, 1, plot);
                            fprintf("Cost after %d samples: %.1f\n", i-2, cost)
                            return
                        end
                    end
                    break;
                end
            end  
        end
    end
    
    % output test data as [J, k, t]
    A = J;
    qs = k;
    cost = t;
    

end

