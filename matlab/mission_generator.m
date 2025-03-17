% Generate mission json file
N_files = 30;
start_points = cell(N_files, 1);
goal_points = cell(N_files, 1);

for iter = 1:N_files
    %% Configuration
%     file_name = ['forest10_spin4_', int2str(iter)];
    file_name = ['maze10_sparse_patrol2_', int2str(iter)];
    max_vel = '[1.0, 1.0, 1.0]'; %[m/s]
    max_acc = '[2.0, 2.0, 2.0]'; %[m/s^2]
    radius = '0.15'; % [m]
    downwash = '2.0';
%     dimension = '[-5.0, -5.0, 0.0, 5.0, 5.0, 2.5]';
%    dimension = '[-2.0, 0.0, 0.0, 10.0, 7.2, 2.5]'; % maze10
%     dimension = '[-3.0, 0.0, 0.0, 11.0, 7.2, 2.5]'; % maze20
%     dimension = '[-2.0, -0.3, 0.0, 6.0, 4.3, 2.5]'; % dense maze 10
    dimension = '[-2.0, -0.3, 0.0, 8.0, 5.8, 2.5]'; % sparse maze 10
    N = 10; % The number of agents
    max_noise = 0.0;
    z_2d = 1;
    
    
    % Set start and goal points of agents
    start_points{iter} = zeros(N,3);
    goal_points{iter} = zeros(N,3);

    % Circle swap
%     circle_radius = 4;
%     for qi = 1:N
%         theta = 2 * pi * (qi-1) / N;
%         start_points{iter}(qi,:) = [circle_radius * cos(theta), circle_radius * sin(theta), z_2d];
%         goal_points{iter}(qi,:) = [-circle_radius * cos(theta), -circle_radius * sin(theta), z_2d];
%     end

    % Random forest
%     start_points{iter} = [4.0,0.0,z_2d;
%                           3.0,2.5,z_2d;
%                           1.0,4.0,z_2d;
%                           -1.0,4.0,z_2d;
%                           -3.0,2.5,z_2d;
%                           -4.0,0.0,z_2d;
%                           -3.0,-2.5,z_2d;
%                           -1.0,-4.0,z_2d;
%                           1.0,-4.0,z_2d;
%                           3.0,-2.5,z_2d];
%     goal_points{iter} = [-4.0,0.0,z_2d;
%                          -3.0,-2.5,z_2d;
%                          -1.0,-4.0,z_2d;
%                          1.0,-4.0,z_2d;
%                          3.0,-2.5,z_2d;
%                          4.0,0.0,z_2d;
%                          3.0,2.5,z_2d;
%                          1.0,4.0,z_2d;
%                          -1.0,4.0,z_2d;
%                          -3.0,2.5,z_2d]; 
    

    % Narrowgap
%     start_points = [-3.0,1.0,z_2d;
%                     -3.0,0.5,z_2d;
%                     -3.0,0.0,z_2d;
%                     -3.0,-0.5,z_2d;
%                     -3.0,-1.0,z_2d;
%                     3.0,1.0,z_2d;
%                     3.0,0.5,z_2d;
%                     3.0,0.0,z_2d;
%                     3.0,-0.5,z_2d;
%                     3.0,-1.0,z_2d];
%     goal_points =  [3.0,-1.0,z_2d;
%                     3.0,-0.5,z_2d;
%                     3.0,0.0,z_2d;
%                     3.0,0.5,z_2d;
%                     3.0,1.0,z_2d;
%                     -3.0,-1.0,z_2d;
%                     -3.0,-0.5,z_2d;
%                     -3.0,0.0,z_2d;
%                     -3.0,0.5,z_2d;
%                     -3.0,1.0,z_2d];         
         
    % maze10
%     start_points{iter} = [-1.0,4.5,z_2d;
%                     -1.0,4.0,z_2d;
%                     -1.0,3.5,z_2d;
%                     -1.0,3.0,z_2d;
%                     -1.0,2.5,z_2d;
%                     9.0,4.5,z_2d;
%                     9.0,4.0,z_2d;
%                     9.0,3.5,z_2d;
%                     9.0,3.0,z_2d;
%                     9.0,2.5,z_2d];
%     goal_points{iter} =  [9.0,2.5,z_2d;
%                     9.0,3.0,z_2d;
%                     9.0,3.5,z_2d;
%                     9.0,4.0,z_2d;
%                     9.0,4.5,z_2d;
%                     -1.0,2.5,z_2d;
%                     -1.0,3.0,z_2d;
%                     -1.0,3.5,z_2d;
%                     -1.0,4.0,z_2d;
%                     -1.0,4.5,z_2d]; 
                
    % maze20
%     start_points{iter} = [-2.0,4.5,z_2d;
%                     -2.0,4.0,z_2d;
%                     -2.0,3.5,z_2d;
%                     -2.0,3.0,z_2d;
%                     -2.0,2.5,z_2d;
%                     -1.0,4.5,z_2d;
%                     -1.0,4.0,z_2d;
%                     -1.0,3.5,z_2d;
%                     -1.0,3.0,z_2d;
%                     -1.0,2.5,z_2d;
%                     10.0,4.5,z_2d;
%                     10.0,4.0,z_2d;
%                     10.0,3.5,z_2d;
%                     10.0,3.0,z_2d;
%                     10.0,2.5,z_2d
%                     9.0,4.5,z_2d;
%                     9.0,4.0,z_2d;
%                     9.0,3.5,z_2d;
%                     9.0,3.0,z_2d;
%                     9.0,2.5,z_2d];
%     goal_points{iter} =  [10.0,2.5,z_2d;
%                     10.0,3.0,z_2d;
%                     10.0,3.5,z_2d;
%                     10.0,4.0,z_2d;
%                     10.0,4.5,z_2d;
%                     9.0,2.5,z_2d;
%                     9.0,3.0,z_2d;
%                     9.0,3.5,z_2d;
%                     9.0,4.0,z_2d;
%                     9.0,4.5,z_2d;
%                     -2.0,2.5,z_2d;
%                     -2.0,3.0,z_2d;
%                     -2.0,3.5,z_2d;
%                     -2.0,4.0,z_2d;
%                     -2.0,4.5,z_2d
%                     -1.0,2.5,z_2d;
%                     -1.0,3.0,z_2d;
%                     -1.0,3.5,z_2d;
%                     -1.0,4.0,z_2d;
%                     -1.0,4.5,z_2d]; 
%                 
    
    % dense maze10
%     start_points{iter} = [-1.0,3.0,z_2d;
%                           -1.0,2.5,z_2d;
%                           -1.0,2.0,z_2d;
%                           -1.0,1.5,z_2d;
%                           -1.0,1.0,z_2d;
%                           5.0,3.0,z_2d;
%                           5.0,2.5,z_2d;
%                           5.0,2.0,z_2d;
%                           5.0,1.5,z_2d;
%                           5.0,1.0,z_2d];
%     goal_points{iter} = [5.0,1.0,z_2d;
%                          5.0,1.5,z_2d;
%                          5.0,2.0,z_2d;
%                          5.0,2.5,z_2d;
%                          5.0,3.0,z_2d;
%                          -1.0,1.0,z_2d;
%                          -1.0,1.5,z_2d;
%                          -1.0,2.0,z_2d;
%                          -1.0,2.5,z_2d;
%                          -1.0,3.0,z_2d]; 

%     sparse maze10
    start_points{iter} = [-1.5, 3.5,z_2d;
                          -1.5, 3.0,z_2d;
                          -1.5, 2.5,z_2d;
                          -1.5, 2.0,z_2d;
                          -1.5, 1.5,z_2d;
                          7, 3.5,z_2d;
                          7, 3.0,z_2d;
                          7, 2.5,z_2d;
                          7, 2.0,z_2d;
                          7, 1.5,z_2d];
    goal_points{iter} = [7, 1.5,z_2d;
                         7, 2.0,z_2d;
                         7, 2.5,z_2d;
                         7, 3.0,z_2d;
                         7, 3.5,z_2d;
                         -1.5, 1.5,z_2d;
                         -1.5, 2.0,z_2d;
                         -1.5, 2.5,z_2d;
                         -1.5, 3.0,z_2d;
                         -1.5, 3.5,z_2d]; 

    % Add noise
    for qi = 1:N
        start_points{iter}(qi,:) = start_points{iter}(qi,:) + (2 * rand([1,3]) - 1.0) * max_noise;
        goal_points{iter}(qi,:) = goal_points{iter}(qi,:) + (2 * rand([1,3]) - 1.0) * max_noise;
    end

    % Set obstacles
    % TODO: Generate obstacles automatically
    N_obs = 2;
    obstacles = cell(N_obs);
%     obstacles{1} = '{"type": "spin", "axis_position": [0.0, 0.0, 1.0], "axis_ori": [0,0,1], "start": [2.0, 0.0, 1.0], "speed": 1.0, "size": 0.3, "max_acc": 1.0, "downwash": 1.0}';
%     obstacles{2} = '{"type": "spin", "axis_position": [0.0, 0.0, 1.0], "axis_ori": [0,0,1], "start": [0.0, 2.0, 1.0], "speed": 1.0, "size": 0.3, "max_acc": 1.0, "downwash": 1.0}';
%     obstacles{3} = '{"type": "spin", "axis_position": [0.0, 0.0, 1.0], "axis_ori": [0,0,1], "start": [-2.0, 0.0, 1.0], "speed": 1.0, "size": 0.3, "max_acc": 1.0, "downwash": 1.0}';
%     obstacles{4} = '{"type": "spin", "axis_position": [0.0, 0.0, 1.0], "axis_ori": [0,0,1], "start": [0.0, -2.0, 1.0], "speed": 1.0, "size": 0.3, "max_acc": 1.0, "downwash": 1.0}';

%     obstacles{1} = '{"type": "spin", "axis_position": [4.0, 4.0, 1.0], "axis_ori": [0,0,1], "start": [6.0, 4.0, 1.0], "speed": 0.5, "size": 0.3, "max_acc": 2.0, "downwash": 1.0}';
%     obstacles{2} = '{"type": "spin", "axis_position": [4.0, 4.0, 1.0], "axis_ori": [0,0,1], "start": [4.0, 6.0, 1.0], "speed": 0.5, "size": 0.3, "max_acc": 2.0, "downwash": 1.0}';
%     obstacles{3} = '{"type": "spin", "axis_position": [4.0, 4.0, 1.0], "axis_ori": [0,0,1], "start": [2.0, 4.0, 1.0], "speed": 0.5, "size": 0.3, "max_acc": 2.0, "downwash": 1.0}';
%     obstacles{4} = '{"type": "spin", "axis_position": [4.0, 4.0, 1.0], "axis_ori": [0,0,1], "start": [4.0, 2.0, 1.0], "speed": 0.5, "size": 0.3, "max_acc": 2.0, "downwash": 1.0}';

%     obstacles{1} = '{"type": "spin", "axis_position": [0.0, 0.0, 1.0], "axis_ori": [0,0,1], "start": [2.0, 0.0, 1.0], "speed": 0.5, "size": 0.3, "max_acc": 2.0, "downwash": 2.0}';
%     obstacles{2} = '{"type": "spin", "axis_position": [0.0, 0.0, 1.0], "axis_ori": [0,0,1], "start": [-2.0, 0.0, 1.0], "speed": 0.5, "size": 0.3, "max_acc": 2.0, "downwash": 2.0}';

%     obstacles{1} = '{"type": "patrol", "waypoints": [{"waypoint":[-0.5, 1.0, 1.0]}, {"waypoint":[4.5, 3.0, 1.0]}], "size": 0.2, "speed": 0.25, "max_acc": 1}';
    
    obstacles{1} = '{"type": "patrol", "waypoints": [{"waypoint":[-0.5, 1.0, 1.0]}, {"waypoint":[6.0, 4.0, 1.0]}], "size": 0.2, "speed": 1.0, "max_acc": 1}';
    obstacles{2} = '{"type": "patrol", "waypoints": [{"waypoint":[6.0, 4.5, 1.0]}, {"waypoint":[-0.5, 1.5, 1.0]}], "size": 0.2, "speed": 1.0, "max_acc": 1}';
    
    %% Generate Mission
    % Add .json to file name
    file_name = [file_name, '.json']; 

    % String for mission 
    agents = '  "agents": [\n';
    type = '    {"type": ';
    crazyflie = '"crazyflie", ';
    cid = '"cid": ';
    start = ', "start": [';
    goal = '], "goal": [';
    next = ']},\n';
    eol = ']}\n  ],\n';
    intro1 = '{\n  "quadrotors": {\n    "crazyflie": {\n    "max_vel":';
    intro2 = ',\n    "max_acc": ';
    intro3 = ',\n    "radius": ';
    intro4 = ',\n    "nominal_velocity": 1.0,\n    "downwash": ';
    intro5 = '},\n  "default": {\n    "max_vel": [1.0, 1.0, 1.0],\n    "max_acc": [2.0, 2.0, 1.0],\n    "radius": 0.15,\n    "nominal_velocity": 1.0,\n    "downwash": 2.0}\n  },\n\n  "world": [\n    {"dimension": ';
    intro6 = '}\n  ],\n\n';
    intro = [intro1, max_vel, intro2, max_acc, intro3, radius, intro4, downwash, intro5, dimension, intro6];
    obs = '\n  "obstacles": [\n';
    for i = 1 : N_obs
        if i < N_obs
            obs = [obs, '    ', obstacles{i}, ',\n'];
        else
            obs = [obs, '    ', obstacles{i}];
        end

    end
    outro = [obs, '\n  ]\n}'];

    % Generate a random set of initial and final positions
    fileID = fopen(file_name,'w');
    fprintf(fileID,intro);
    mission = agents;
    for qi = 1:N
        start_point = [num2str(start_points{iter}(qi,1)), ', ', ...
                       num2str(start_points{iter}(qi,2)), ', ', ...
                       num2str(start_points{iter}(qi,3))];
        goal_point = [num2str(goal_points{iter}(qi,1)), ', ', ...
                      num2str(goal_points{iter}(qi,2)), ', ', ...
                      num2str(goal_points{iter}(qi,3))];

        mission = [mission, type, crazyflie, cid, num2str(qi), start, start_point, goal, goal_point];
        if(qi < N)
            mission = [mission, next];
        else
            mission = [mission, eol];
        end
    end
    fprintf(fileID,mission);
    fprintf(fileID,outro);

    fclose(fileID);
end
