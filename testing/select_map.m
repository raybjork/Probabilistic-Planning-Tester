% this function returns a map struct given a map number
function map = select_map(i)
    map.bounds = [0 100 0 100];
    map.goal_r = 3;
    switch i
        case 1
            map.obstacles = [[30, 10, 10, 80]; ...
                             [55, 40, 15, 30]];
            map.start = [5 5];
            map.goal = [95 95];
        case 2
            map.obstacles = [[40, 51, 20, 20]; ...
                             [40, 29, 20, 20]];
            map.start = [30 50];
            map.goal = [70 50];
        case 3
            map.start = [5 5];
            map.goal = [95 95];

            map.obstacles = [[10, 0, 10, 90]; ...
                             [30, 10, 10, 90];
                             [60, 0, 10, 90]; ...
                             [80, 10, 10, 90]];
        case 4
            map.obstacles = [-10, -10, 10, 10];
            map.start = [25 75];
            map.goal = [75 25];
        case 5
            w = 5;
            h = 5;
            num_obstacles = 60;
            map.start = [15 85];
            map.goal = [85 15];
            map.obstacles = zeros(num_obstacles, 4);
            for i = 1 : num_obstacles
                map.obstacles(i,:) = [rand*95, rand*95, w,h];
                while obstacle_intersect(map.obstacles(i, :), map.start) ...
                   || obstacle_intersect(map.obstacles(i, :), map.goal)
                    map.obstacles(i,:) = [rand*95, rand*95, w,h];
                end
                        
            end
        case 6
            map.goal_r = 5;
            map.start = [50 50];
            map.goal = [95 95];
            map.obstacles = [-10, -10, 10, 10];
        case 7
            map.obstacles = [[40, 53, 20, 210]; ...
                             [40, 0, 20, 47]];
            map.start = [25 75];
            map.goal = [75 25];
    end
end

function bad = obstacle_intersect(o, q)
    if o(1)+o(3)>q(1) && o(1) < q(1) && ...
       o(2)+o(4)>q(2) && o(2) < q(2)
        bad = true;
    else
        bad = false;
    end
end
