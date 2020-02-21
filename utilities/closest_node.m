function [index, dist] = closest_node(tree, configurations, query)
%% CLOSEST NODE 
%   Given a query node, search a tree for its closest neighbor 
%
% INPUTS:
%   TREE            - adjacency matrix corresponding to a tree where matrix
%                     element TREE(i,j) represents a bidirectional edge 
%                     between node index i and j, and TREE(i,i) represents
%                     a node having membership in the tree
%   CONFIGURATIONS  - a list of all sampled nodes (note: does not say 
%                     anything about a node's edges or if its in TREE)
%   QUERY           - index of query node in CONFIGURATIONS list
%
%
% OUTPUT: index of closest node to QUERY node in the CONFIGURATIONS list
% 
% AUTHOR
%   Ray Bjorkman - raybjork@seas.upenn.edu
%   Akhil Devarakonda - adevara@seas.upenn.edu

    %% setup
    % initialize minimum distance to be infinity 
    min_dist = inf;     

    % get each node from the adjacency matrix by looking on its diagonal
    nodes = find(diag(tree));  

    %% check each node in tree and find closest
    % Euclidean distance used as metric for comparison
    for i = 1:length(nodes) 
        dist = norm(configurations(nodes(i), :) - configurations(query, :));
        if dist < min_dist
            min_dist = dist;
            index = nodes(i);
        end
    end
end
