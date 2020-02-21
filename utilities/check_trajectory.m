function [valid, collisions] = check_trajectory(q1, q2, O)
%% CHECK_TRAJECTORY 
%   Determine if the interpolated line between two robot
%   configurations is valid
%
% INPUTS:
%   q1          - vector of the starting configuration
%   q2          - vector of the goal configuration
%   o           - column vector of boundary boxes of obstacles
%
% OUTPUT: intersections - 2D matrix of xy points the trajectory-obstacle
%                         intersections - will be empty if none
%                         
% AUTHOR
%   Ray Bjorkman - raybjork@seas.upenn.edu

    %% setup
    collisions = [];
    valid = true;

    % generate line for trajectory
    xline = [q1(1) q2(1)];
    yline = [q1(2) q2(2)];

    %% check each node on trajectory of line
    % check if it leaves robot's limits
    % check if it collides with obstacles or the boundary
    for i = 1:size(O, 1)
        % generate rectangular lines for each obstacle
        xbox = [O(i, 1) O(i, 1) O(i, 1) + O(i, 3) O(i, 1) + O(i, 3) O(i, 1)];
        ybox = [O(i, 2) O(i, 2) + O(i, 4) O(i, 2) + O(i, 4) O(i, 2) O(i, 2)];

        % compute intersections
        [x, y] = polyxpoly(xline, yline, xbox, ybox);
        if ~isempty(x)
            collisions = vertcat(collisions, [x, y]);
            valid = false;
        end
    end 
end