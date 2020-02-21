function [neighbors] = k_nearest(A, configurations, query, radius, start_i)
%% CLOSEST NODE 
%   Given a query node, search a tree for its closest neighbors
%
% INPUTS:
%   TREE            - adjacency matrix corresponding to a tree where matrix
%                     element TREE(i,j) represents a bidirectional edge 
%                     between node index i and j, and TREE(i,i) represents
%                     a node having membership in the tree
%   CONFIGURATIONS  - a list of all sampled nodes (note: does not say 
%                     anything about a node's edges or if its in TREE)
%   QUERY           - index of query node in CONFIGURATIONS list
%   RADIUS          - radius around query node to search
%
%
% OUTPUT: index of best neighbors of QUERY node in the CONFIGURATIONS list,
%         ordered in terms of how good they are
% 
% AUTHOR
%   Ray Bjorkman - raybjork@seas.upenn.edu


    %% setup
    % get each node from the adjacency matrix by looking on its diagonal
    nodes = find(diag(A));  
    
    % array of near neighbors
    neighbors = [];
    population = 0;
    
    %% check each node in tree and near neighbors
    for i = 1:length(nodes) 
        % Euclidean distance used as metric for comparison
        d1 = norm(configurations(nodes(i), :) - configurations(query, :));
        if d1 <= radius
            population = population + 1;
            d2 = A(nodes(i), nodes(i));
            neighbors(population, :) = [fix(nodes(i)) ; d1 + d2];
        end
    end
    if ~isempty(neighbors)
        neighbors = sortrows(neighbors, 2);
    end
end
