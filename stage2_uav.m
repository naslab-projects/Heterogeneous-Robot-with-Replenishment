clear
close all
tic;
%% Configure mission area

% First, load the boat stationary point as the charging point
% Drone execute the Cover Path Planning by GA
% Choose the closest charging point
% Assume every boat has two charging slot and we have two drones

%% Load data

% charger position [start 1 2 3] 1x4
load('charger1.mat');
asv1_x = asv_x;
asv1_y = asv_y;
asv1_xy = [asv1_x asv1_y];

clear asv_x asv_y;
load('charger2.mat');
asv2_x = asv_x;
asv2_y = asv_y;
asv2_xy = [asv2_x asv2_y];

load('asv_travel.mat');


% calculateDistance(x1, y1, x2, y2)
asv1_distance_from_center = calculateDistance( asv1_x(2), asv1_y(2), 7, 7);  % choose 2nd point, because we do not use start point
asv2_distance_from_center = calculateDistance( asv2_x(2), asv2_y(2), 7, 7);

asv1_is_closer_to_center_than_asv2 = 0;
if asv1_distance_from_center < asv2_distance_from_center   % asv1 is closer to the center than asv2, so asv1 bring two workers
    worker_1_x = asv1_x(2); % asv1
    worker_1_y = asv1_y(2); 
    worker_2_x = asv1_x(2); % asv1
    worker_2_y = asv1_y(2);
    worker_3_x = asv2_x(2); % asv2
    worker_3_y = asv2_y(2);
    asv1_is_closer_to_center_than_asv2 = 1;
else % asv2 is closer to the center than asv1, so asv2 bring two workers
    worker_1_x = asv1_x(2); % asv1
    worker_1_y = asv1_y(2);
    worker_2_x = asv2_x(2); % asv2
    worker_2_y = asv2_y(2);
    worker_3_x = asv2_x(2); % asv2
    worker_3_y = asv2_y(2);
    asv1_is_closer_to_center_than_asv2 = 2;
end

%%

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
v = 20/scala; %20/scala; %3/scala; % worker speed   3km/hr with 2hr safety level   % change the drone speed 
vc = 16/scala; % 2/scala; % charger speed  16km/hr
numCharger = 2; % number of worker                           % when load data we will konw 
numWorker = 3; % number of charger                           % change the worker number 
btl = 0.5; % 0.4 % 2; %8, 10; % battery life  max 12 hr battery 
uav_battery_limit = 0.7; % hour => 42 min
charging_period = 2; % 8; % charging period hr 
charging_period_drone = 2; % % charging period hr 
%% Add start point
%start_point = [8,0;8,0;8,0;8,0]; % x and y locations of start points of workers

start_point = [worker_1_x, worker_1_y; 
               worker_2_x, worker_2_y;
               worker_3_x, worker_3_y;
               8,0]; % x and y locations of start points of workers

% start_point = [8,0;8,0;8,0;8,0;8,0]; % x and y locations of start points of workers
start_pCharger = [8,0;8,0;8,0;8,0]; % x and y locations of start points of chargers
start_mat = zeros(length(map.mission_location),numWorker*2+numCharger);

% Calculate distance to start point
for start_i = 1:numWorker
    for is = 1: length(map.mission_location)
        start_mat(is,(start_i-1)*2+1) = norm([map.mission_location(is,:)-start_point(start_i,:)]);
        start_mat(is,(start_i-1)*2+2) = v;
    end
end

% add charger initial positions
for start_i = 1: numCharger
    for is = 1: length(map.mission_location)
        start_mat(is,2*numWorker+start_i) = norm([map.mission_location(is,:)-start_pCharger(start_i,:)]);
    end
end
%% GA parameters
userConfig.xy = map.mission_location;              % mission area
userConfig.minTour=floor(size(userConfig.xy,1)/1); % Assume all the vehicle will travel the similar distance.
userConfig.popSize=400*2;                          % population size
userConfig.numIter = 100; %400*2;                        % number of max iteration
% userConfig.numIter = 1;                          % number of max iteration
userConfig.nSalesmen= numWorker;                   % number of worker
userConfig.numTarChargers = numCharger;            % number of charger
userConfig.batteryLife = btl;
userConfig.initial_battery_level = btl;   % 100% energy
userConfig.charging_time = charging_period;
userConfig.charging_time_drone = charging_period_drone;
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
userConfig.asv1_xy = asv1_xy;
userConfig.asv2_xy = asv2_xy;
userConfig.uav_return_speed = 60; % 60 km/hr
userConfig.uav_battery_limit = uav_battery_limit;
userConfig.asv_travel = asv_travel;
userConfig.asv1_is_closer_to_center_than_asv2 = asv1_is_closer_to_center_than_asv2;
% Create a struct C with fields 'data' to store matrices A and B
asv_xy(1).data = asv1_xy;
asv_xy(2).data = asv2_xy;

%% lifan add
run_times = 100; 
record_max_mission_time = zeros(run_times,1);
record_travel_distance = zeros(run_times,1);
record_return_distance = zeros(run_times,1);
record_a = cell(run_times,1);

%% Run GA function
if 1 % "1" for single run, "0" for multiple runs using parallel computing
    
    for i = 1:run_times
        %a = ga6_0_drone_CPP(userConfig); % use instant charge and speed limit constraint
        a = stage2_uav_planner(userConfig);
        Run_time = toc/60
        
        record_max_mission_time(i) = max(a.minTime);  
        record_travel_distance(i) = a.minDist; % working robot distance
        record_return_distance(i) = a.minds;  % uav return distance
        record_penalty(i) = a.min_uav_cannot_fly_back_penalty;
        record_a{i} = a;
        fprintf('record_max_mission_time ( %d ) = %.1f record_max_distance = %.1f  \n', i, record_max_mission_time(i), record_travel_distance(i)); % lifan add
    end
    
    std_mission_time = std(record_max_mission_time);
    mean_mission_time = mean(record_max_mission_time);
    
    %%
    % Find indices where a is equal to 0, check there is no penalty 
    zero_indices = find(record_penalty == 0);
    % Select the maximum value's index from b at those positions
    b = record_max_mission_time;
    min_index = zero_indices(b(zero_indices) == min(b(zero_indices)));
    %[min_mission_time min_index] = min(record_max_mission_time);   % choose the best one , and choose the uav_cannot_fly_back_penalty_record = 0
    %%
    
    
    min_mission_time = record_max_mission_time(min_index);
    % update a
    a = record_a{min_index};
    
    std_distance = std(record_travel_distance);
    mean_distance = mean(record_travel_distance);
    
    std_return_distance = std(record_return_distance);
    mean_return_distance = mean(record_return_distance);
    
    % Get the current date and time in a specific format
    currentDateTime = datestr(now, 'yyyy-mm-dd_HHMMSS');
    % Create your filename using the current date and time
    filename = ['uav_time_sequence_charging_', currentDateTime, '.mat'];
    
    % Save best data 'a' with the generated filename
    save(filename, 'a', 'record_max_mission_time', 'record_travel_distance', 'record_return_distance', 'record_penalty','record_a', ...
                   'min_mission_time', 'min_index',...
                   'std_mission_time', 'mean_mission_time', 'std_distance', 'mean_distance',...
                   'std_return_distance', 'mean_return_distance', ...
                   'numCharger', 'userConfig', 'start_point', 'start_pCharger', 'charging_period','asv_xy');
    
 
    % Load data, after load data run 'preplan_plot_6_drone_CPP' and then run 'test_plot_uav_6'
    %load('uav_time_sequence_charging_2024-01-20_051116');
    %load('asv_travel.mat');
               
    
    % Evaluate of the trajectories and get the timeline and plot
    %[traj_segment,time_given_charger] = preplan_plot4_0_drone_CPP(a,userConfig.numTarChargers,userConfig,start_point,start_pCharger);
    %[traj_segment] = preplan_plot_6_drone_CPP(a,userConfig.numTarChargers,userConfig,start_point,start_pCharger, asv_xy);
    [traj_segment] = stage2_uav_plot_path(a,userConfig.numTarChargers,userConfig,start_point,start_pCharger, asv_xy);
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
    %[traj_segment,time_given_charger] = preplan_plot4_0(a,userConfig.numTarChargers,userConfig,start_point,start_pCharger);
    [traj_segment,time_given_charger] = preplan_plot4_1(a,userConfig.numTarChargers,userConfig,start_point,start_pCharger, asv_xy);
end


function distance = calculateDistance(x1, y1, x2, y2)
    distance = sqrt((x2 - x1)^2 + (y2 - y1)^2);
end