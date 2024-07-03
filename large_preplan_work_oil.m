clear 
close all
tic;
%% Configure mission area
map1 = Map_mh('oilspill_3.png','resolution',2);
map = Map_mh('oilspill_3.png','resolution',2);
represent(map)
lawn_mower(map)

%% Speed and mission specifications
scala = 1; 
v = 3/scala;
vc = 6/scala;

delta_v = v;
delta_vc = vc;

numCharger = 4;
numWorker = 7;
btl = 10;
charging_period = 3;

%% get starting time
temp_matr = [0 0 0];
max_btl = 9;
start_levels = btl-repmat(temp_matr,1,max_btl);

%% Add start point (need a function)

start_point = [40,40];
start_pCharger = [40,40;40,40;40,40;40,40;40,40;40,40;40,40];
start_mat = zeros(length(map.mission_location),1*2+numCharger);

% Calculate distance to start point
for start_i = 1:1
    for is = 1: length(map.mission_location)
        start_mat(is,(start_i-1)*2+1) = norm([map.mission_location(is,:)-start_point(start_i,:)]);
        start_mat(is,(start_i-1)*2+2) = v;
    end
end

% add charger initial positions
for start_i = 1: 1
    for is = 1: length(map.mission_location)
        start_mat(is,2*1+start_i) = norm([map.mission_location(is,:)-start_pCharger(start_i,:)]);
    end
end
%% GA parameters

userConfig.xy = map.mission_location; % mission area
userConfig.minTour=floor(size(userConfig.xy,1)/1); % Assume all the vehicle will travel the similar distance.
userConfig.popSize=400*2*20; % population size
userConfig.numIter = 600*20;     % number of max iteration
% userConfig.numIter = 1;     % number of max iteration
userConfig.nSalesmen= numWorker;  % number of worker
userConfig.numTarChargers = numCharger;    % number of charger
userConfig.batteryLife = btl;    % interval
userConfig.initial_battery_level = start_levels(1:userConfig.nSalesmen);
userConfig.charging_time = charging_period;
userConfig.start_mat = start_mat;
% speed
userConfig.delta_v = delta_v;
userConfig.delta_vc = delta_vc;

% weights
userConfig.alpha_ratio = zeros(userConfig.numIter,5);
userConfig.alpha_ratio(:,1) = .2/3*1;

userConfig.alpha_ratio(1:3000,5) = (1:3000)*.0013;
userConfig.alpha_ratio(3001:6000,5) = (3000:-1:1)*.0012;
userConfig.alpha_ratio(6001:9000,5) = (1:3000)*.0013;
userConfig.alpha_ratio(9001:end,5) = .4;

% userConfig.alpha_ratio(1:end,5) = .4;
userConfig.showWaitbar = true; 

% initial population
one_solution = 1:length(map.mission_location);
initial_pop = repmat(one_solution,userConfig.popSize,1);
userConfig.initial_pop = initial_pop;
userConfig.charging_window = 0;
break1 = cumsum(ones(1,numWorker-1)*floor(size(userConfig.xy,1)/numWorker));
userConfig.break1 = break1;

%% Run GA function
a = ga_oil(userConfig); % use instant charge and speed limit constraint
Run_time = toc/60
save('2_23_oil_1arge1')
run large2_preplan_work_oil.m
%% Evaluate of the trajectories and get the timeline and plot

% [traj_segment,time_given_charger] = preplan_plot_oil(a,userConfig.numTarChargers,userConfig,start_point,start_pCharger);
% disp(a.charger_travel_time)
% a.time_given_charger
% a.minTime