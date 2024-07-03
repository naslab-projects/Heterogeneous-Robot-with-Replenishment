clear
close all
tic;
%% Configure mission area
if 1 % "1" for structured mission area, and "0" for unstructured mission area
    % structured mission area
    map = Map;
    % define mission area length and resolution
    area_x = 1;
    area_y = 1;
    area_length_x = 13;
    area_length_y = 13;
    map.res = 1;
    
    x = area_x:map.res:(area_x + area_length_x );
    y = area_y:map.res:(area_y + area_length_y );
    
    [X,Y] = meshgrid(x,y);
    
    X = reshape(X,[],1);
    Y = reshape(Y,[],1);
    
    mission_location = [X Y];
    just_matrix = false(area_x + max(area_length_x,area_length_y));
    just_matrix(1:x(end),1:y(end))=1;
    % add island 
    % just_matrix(6:15,6:15)=0;
    
    
    map.mission_location = mission_location;
    map.matrix = just_matrix;
    
    represent(map);
    lawn_mower(map);
    % unstructured mission area (need to load a picture)
else
    map1 = Map_mh('map_pic.png','resolution',8);
    map = Map_mh('map_pic.png','resolution',8);
    represent(map)
    lawn_mower(map)
end
%% Speed and mission specifications
scala = 1; % scale based on mission area resolution
v = 3/scala; % worker speed   3km/hr with 2hr safety level
vc = 16/scala; % 2/scala; % charger speed  16km/hr
numCharger = 2; % number of charger
numWorker = 3; % number of worker
btl = 10; % battery life  max 12 hr battery 
charging_period = 8; % 3; % charging period hr 

vd = 3/scala; % worker speed, drone.
numDrone = 3;

%% Add start point
start_point = [8,0;8,0;8,0;8,0]; % x and y locations of start points of workers
% start_point = [8,0;8,0;8,0;8,0;8,0]; % x and y locations of start points of workers
start_pCharger = [8,0;8,0;8,0;8,0]; % x and y locations of start points of chargers
start_mat = zeros(length(map.mission_location),numWorker*2+numCharger);

% Calculate distance to start point 196 x 6    3 working robot (dist v) (dist v) (dist v)
for start_i = 1:numWorker
    for is = 1: length(map.mission_location)
        start_mat(is,(start_i-1)*2+1) = norm([map.mission_location(is,:)-start_point(start_i,:)]);  % 196 point to the start point distance
        start_mat(is,(start_i-1)*2+2) = v;
    end
end

% add charger initial positions
for start_i = 1: numCharger
    for is = 1: length(map.mission_location)
        start_mat(is,2*numWorker+start_i) = norm([map.mission_location(is,:)-start_pCharger(start_i,:)]); % 196 point to the start point distance
    end
end
%% add boundary
ob = map.mission_location;
% points_to_remove = [6 4; 6 5; 6 6; 6 7; 7 4; 7 5; 7 6; 8 5; 8 6]; % island
% points_to_remove = [6 5; 6 6; 6 7; 7 4; 7 5; 7 6; 8 5; 8 6; 9 4; 9 5]; % island test 1

points_to_remove = [5 5; 5 6; 6 4; 7 4; 9 4; 10 4; 6 5; 7 5; 8 5; 9 5; 6 6; 7 6; 8 6; 7 7; 8 7; 9 6; 9 7; 9 8; 10 7];  % 



% points_to_remove = [1 1; 1 2; 1 3; 1 4; 1 5; 1 6; 1 7; 1 8; 1 9; 1 10; 1 11; 1 12; 1 13; 1 14;
%                     2 1; 2 2; 2 3; 2 4; 2 5; 2 6; 2 7; 2 8; 2 9; 2 10; 2 11; 2 12; 2 13; 2 14;];
% Create logical index
logical_index = ismember(ob, points_to_remove, 'rows');
% Remove rows based on logical index
ob(logical_index, :) = [];
map.mission_location = ob ;

%% add start_mat boundary to recalulate 196x6 start_mat

% Create a 14x14 matrix
matrix = zeros(14, 14);
% Populate the matrix with consecutive numbers in each row
for i = 1:14
    matrix(i, :) = (i-1)*14 + 1 : i*14;
end
% Display the matrix
% disp('Matrix:');
% disp(matrix);

% Initialize arrays for s, t, and weight
s = [];
t = [];
weight = [];

% Iterate through the matrix to define connections
for i = 1:size(matrix, 1)
    for j = 1:size(matrix, 2)
        % Current node
        current_node = matrix(i, j);
        
        % Check right neighbor
        if j < size(matrix, 2)
            right_neighbor = matrix(i, j+1);
            s = [s current_node];
            t = [t right_neighbor];
            weight = [weight 1];
        end
        
        % Check lower neighbor
        if i < size(matrix, 1)
            lower_neighbor = matrix(i+1, j);
            s = [s current_node];
            t = [t lower_neighbor];
            weight = [weight 1];
        end
        
        % Check lower-right neighbor (diagonal)
        if i < size(matrix, 1) && j < size(matrix, 2)
            lower_right_neighbor = matrix(i+1, j+1);
            s = [s current_node];
            t = [t lower_right_neighbor];
            weight = [weight sqrt(2)]; % Diagonal distance
        end
        
        % Check lower-left neighbor (diagonal)
        if i < size(matrix, 1) && j > 1
            lower_left_neighbor = matrix(i+1, j-1);
            s = [s current_node];
            t = [t lower_left_neighbor];
            weight = [weight sqrt(2)]; % Diagonal distance
        end
    end
end

% Initialize arrays for x and y
x = [];
y = [];
% Number of rows and columns in the matrix
num_rows = 14;
num_cols = 14;
% Populate x and y arrays
for i = 1:num_rows
    for j = 1:num_cols
        x = [x j];
        y = [y i];
    end
end
% add start point new node
x = [x 8];
y = [y 0];
start_node_th = num_rows*num_cols+1; % put the start node in the end
s = [s start_node_th start_node_th start_node_th];
t = [t 7 8 9];
weight = [weight 1.414 1 1.414];



increaseFactor = 99;
remove_node_arr = (points_to_remove(:,2)-1)*14+points_to_remove(:,1);% (y-1)*14+x = nth 

for i = 1:length(remove_node_arr)
    nodesToSelected = remove_node_arr(i);
    [s, t, weight] = increase_weights(s, t, weight, nodesToSelected, increaseFactor);
end
scatter(points_to_remove(:,1),points_to_remove(:,2),'MarkerFaceColor',[0 .7 .7])
hold on


% test point
start_p = [13 7];
end_p = [11 9];
start_p_indx = start_p(1)+(start_p(2)-1)*14;
end_p_indx = end_p(1)+(end_p(2)-1)*14;

% demo
% [path1,d] = shortestpath(G,start_p_indx,197) % [path1,d] = shortestpath(G,3,15)
% highlight(p,path1,'EdgeColor','g','LineWidth',3)


% Create the initial graph
G = graph(s,t,weight);
% Initialize the plot outside the loop
p = plot(G,'XData',x,'YData',y,'EdgeLabel',G.Edges.Weight);


%% boundary II : start_mat
% Update distance to start point 196 x 6    3 working robot (dist v) (dist v) (dist v)
for start_i = 1:numWorker
    for is = 1: length(map.mission_location) %196
        %start_mat(is,(start_i-1)*2+1) = norm([map.mission_location(is,:)-start_point(start_i,:)]);  % 196 point to the start point distance
        % map.mission_location(is,1) = x
        % map.mission_location(is,2) = y
        nth_node = (map.mission_location(is,2)-1)*14+map.mission_location(is,1);% (y-1)*14+x = nth 
        % Find the shortest path
        [path1, d] = shortestpath(G, nth_node, 197);        
        start_mat(is,(start_i-1)*2+1) = d;
        %start_mat(is,(start_i-1)*2+2) = v;  % already generate
        
%         % Highlight the path animation 
%         highlight(p, path1, 'EdgeColor', 'g', 'LineWidth', 3);
%         title([ sprintf('%d', nth_node) ' nth node of Distance: '  sprintf('%.2f', d)  ' meter']);
%         % Pause to observe the plot (optional)
%         pause(0.2);
    end
end

% Update charger initial positions
for start_i = 1: numCharger
    for is = 1: length(map.mission_location)
        %start_mat(is,2*numWorker+start_i) = norm([map.mission_location(is,:)-start_pCharger(start_i,:)]); % 196 point to the start point distance
        nth_node = (map.mission_location(is,2)-1)*14+map.mission_location(is,1);% (y-1)*14+x = nth 
        % Find the shortest path
        [path1, d] = shortestpath(G, nth_node, 197);   
        start_mat(is,2*numWorker+start_i) = d;
    end
end

%% boundary III : dmat 196x196 or 187x187
numPoints = length(map.mission_location)
% Initialize a matrix to store distances
distanceMatrix = zeros(numPoints, numPoints);


% Loop through each unique pair of points and calculate the distance
for i = 1:numPoints  % 1~187
    for j = i+1:numPoints  % 1~187
        
        % map.mission_location(i,1) = x
        % map.mission_location(i,2) = y        
        nth_node = (map.mission_location(i,2)-1)*14+map.mission_location(i,1);% (y-1)*14+x = nth 
        jth_node = (map.mission_location(j,2)-1)*14+map.mission_location(j,1);
        
        % Find the shortest path
        [path1, d] = shortestpath(G, nth_node, jth_node);  
       
        % Fill in the distances for both (i, j) and (j, i)
        distanceMatrix(i, j) = d;
        distanceMatrix(j, i) = d;
    end
end
userConfig.dmat = distanceMatrix;
%% GA parameters
userConfig.xy = map.mission_location;              % mission area
userConfig.minTour=floor(size(userConfig.xy,1)/1); % Assume all the vehicle will travel the similar distance.
userConfig.popSize=400*2;                          % population size
userConfig.numIter = 400*2;                        % number of max iteration
% userConfig.numIter = 1;                          % number of max iteration
userConfig.nSalesmen= numWorker;                   % number of worker
userConfig.numTarChargers = numCharger;            % number of charger
userConfig.batteryLife = btl;
userConfig.initial_battery_level = btl;   % 100% energy
userConfig.charging_time = charging_period;
userConfig.start_mat = start_mat;
% speed
userConfig.delta_v = v;
userConfig.delta_vc = vc;
% weights
userConfig.alpha_ratio = zeros(userConfig.numIter,5);
userConfig.alpha_ratio(:,1) = 1.8;
temp_w = round(userConfig.numIter/4);
temp_w1 = userConfig.alpha_ratio(1,1)/5;
userConfig.alpha_ratio(1:temp_w,5) = (1:temp_w)*temp_w1/temp_w;
userConfig.alpha_ratio(temp_w+1:temp_w*2,5) = (temp_w:-1:1)*temp_w1/temp_w;
userConfig.alpha_ratio(temp_w*2+1:temp_w*3,5) = (1:temp_w)*temp_w1/temp_w;
userConfig.alpha_ratio(temp_w*3+1:end,5) = temp_w1;
userConfig.showWaitbar = true;
% initial population
one_solution = 1:length(map.mission_location);
initial_pop = repmat(one_solution,userConfig.popSize,1);
userConfig.initial_pop = initial_pop;
userConfig.charging_window = 0;
break1 = cumsum(ones(1,numWorker-1)*floor(size(userConfig.xy,1)/numWorker));
userConfig.break1 = break1;


run_times = 1; 
record_max_mission_time = zeros(run_times,1);
record_max_distance = zeros(run_times,1);
record_a = cell(run_times,1);

%% Run GA function
if 1 % "1" for single run, "0" for multiple runs using parallel computing
    
    for i = 1:run_times
        %a = ga5_0(userConfig); % use instant charge and speed limit constraint
        %a = ga6_0(userConfig); % 2024 Feb 12
        a = stage1_auv_planner(userConfig); % 2024 May 13
        Run_time = toc/60
        record_max_mission_time(i) = max(a.minTime);  
        record_max_distance(i) = a.minDist; % working robot distance
        record_a{i} = a;
        
        fprintf('record_max_mission_time ( %d ) = %.1f record_max_distance = %.1f  \n', i, record_max_mission_time(i), record_max_distance(i)); % lifan add
    end
    
    std_mission_time = std(record_max_mission_time);
    mean_mission_time = mean(record_max_mission_time);
    [min_mission_time min_index] = min(record_max_mission_time);
    % update a
    a = record_a{min_index};
    std_distance = std(record_max_distance);
    mean_distance = mean(record_max_distance);
    
    % Get the current date and time in a specific format
    currentDateTime = datestr(now, 'yyyy-mm-dd_HHMMSS');
    % Create your filename using the current date and time
    filename = ['auv_mobile_charger_', currentDateTime, '.mat'];
    
    % Save best data 'a' with the generated filename
    save(filename, 'a', 'record_max_mission_time', 'record_max_distance', 'std_mission_time', 'mean_mission_time', 'std_distance', 'mean_distance',...
                   'numCharger', 'userConfig', 'start_point', 'start_pCharger', 'G', 'start_node_th', 'points_to_remove', 'charging_period');
    
    % Load data, after load data run 'preplan_plot_5_0' and then run 'plot_battery_v10'
    %load('auv_mobile_charger_2024-01-17_162755.mat');

    
    % Evaluate of the trajectories and get the timeline and plot
    % [traj_segment,time_given_charger, traj_worker_nth] = preplan_plot5_0(a,userConfig.numTarChargers,userConfig,start_point,start_pCharger,G,start_node_th,points_to_remove);
    [traj_segment,time_given_charger, traj_worker_nth] = stage1_auv_plot_path(a,userConfig.numTarChargers,userConfig,start_point,start_pCharger,G,start_node_th,points_to_remove);

    %[traj_segment,time_given_charger] = preplan_plot4_0(a,userConfig.numTarChargers,userConfig,start_point,start_pCharger);
    
    
    
    disp(a.charger_travel_time);
    a.time_given_charger;
    a.minTime;
else
    % Monte Carlo Simulation
    monte_iter = 10;
    aM = cell(1,monte_iter);
    
    parfor i = 1:monte_iter
        a = ga4_0(userConfig);
        aM{i} = a;
    end
    save('file_name')
    Run_time = toc/60
    
    scidx = [];
    flidx = [];
    worker_delay = zeros(1,10);
    for iim = 1:monte_iter
        a = aM{iim};
        mission_time_M(iim) = a.minEnergy;
        distance_M(iim) = a.minDist;
        tolerance_M(iim) = a.trackDp(end);
        % check success rate
%         judger = a.charger_travel_time > a.time_given_charger;
%         if sum(judger) == 0
%             scidx = [scidx iim];
%         else
%             flidx = [flidx iim];
%         end
        
    end
%     mean(distance_M(flidx));
%     mean(mission_time_M(flidx));
%     std(distance_M(flidx));
%     std(mission_time_M(flidx));

    [~,temp_I] = min(mission_time_M);
    a = aM{temp_I};
    [traj_segment,time_given_charger] = preplan_plot4_0(a,userConfig.numTarChargers,userConfig,start_point,start_pCharger);
end


%% save file
% Create a file name using the current value of 'i'
fileName = ['a.mat'];
save(fileName, 'a');


function [s, t, weight] = increase_weights(s, t, weight, nodesToSelected, increaseFactor)
    % Find indices of connections involving the specified node
    indicesToIncrease = ismember(s, nodesToSelected);
    % Find indices of connections involving the specified node
    indicesToIncrease_t = ismember(t, nodesToSelected);

    % Increase the weights of connections associated with the specified node
    weight(indicesToIncrease) = increaseFactor;
    weight(indicesToIncrease_t) = increaseFactor;
end