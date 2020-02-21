    % Add that folder plus all subfolders to the path.
    folder = fileparts(which(mfilename)); 
    addpath(genpath(folder));
    
    % Run tests
    [J, K, T] = rrt_comparer(@rrt, 7, 20);
