% trajectory smoothing function for RRT*-Smart
function [A, plot, path, cost] = optimize_path(A, qs, path, O, plot)
    
    % if the path has changed since last iteration, continue
    deleted_nodes = 1;
    while (any(deleted_nodes))
        
        % reset state change array
        deleted_nodes = zeros(length(path), 1);
        
        % iterate through the path
        for i = 1:length(path) - 2
            
            % skip deleted nodes
            if deleted_nodes(i) == 1
                continue;
            end
            
            % check trajectory btw node and grandchild - connect if able
            if check_trajectory(qs(path(i), :), qs(path(i + 2), :), O)
                deleted_nodes(i + 1) = 1; 
            end
        end
        
        % update path
        path = path(deleted_nodes == 0);
    end
    
    % calculate cost
    cost = sum(vecnorm(diff(qs(path,:))'));
end