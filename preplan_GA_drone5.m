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
start_point = [8,0;8,0;8,0;8,0]; % x and y locations of start points of workers
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
userConfig.numIter = 10; %400*2;                        % number of max iteration
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
userConfig.uav_return_speed = 60; %60; % 60 km/hr
userConfig.uav_battery_limit = uav_battery_limit;
userConfig.asv_travel = asv_travel;

% Create a struct C with fields 'data' to store matrices A and B
asv_xy(1).data = asv1_xy;
asv_xy(2).data = asv2_xy;
%% Run GA function
if 1 % "1" for single run, "0" for multiple runs using parallel computing
    a = ga5_0_drone_CPP(userConfig); % use instant charge and speed limit constraint
    Run_time = toc/60
    % Evaluate of the trajectories and get the timeline and plot
    %[traj_segment,time_given_charger] = preplan_plot4_0_drone_CPP(a,userConfig.numTarChargers,userConfig,start_point,start_pCharger);
    [traj_segment] = preplan_plot4_3_drone_CPP(a,userConfig.numTarChargers,userConfig,start_point,start_pCharger, asv_xy);
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