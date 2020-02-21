% Add that folder plus all subfolders to the path.
folder = fileparts(which(mfilename)); 
addpath(genpath(folder));

% Run tests
[J, K, T] = tester(@rrt_star,@rrt, 7, 1);