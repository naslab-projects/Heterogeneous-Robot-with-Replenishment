clear 
close all
tic;
%% Configure mission area
map1 = Map_mh('oilspill_3.png','resolution',8);
map = Map_mh('oilspill_3.png','resolution',8);
represent(map)
lawn_mower(map)
 if true
    % structured mission area
    map = Map;
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
    
    map.mission_location = mission_location;
    map.matrix = just_matrix;
    
    represent(map)
    lawn_mower(map);
    
    % unstructured mission area
else
    map1 = Map_mh('oilspill_3.png','resolution',8);
    map = Map_mh('oilspill_3.png','resolution',8);
    represent(map)
    lawn_mower(map)
end

%% Speed and mission specifications
scala = 5; 
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
% start_levels = btl;

%% Add start point 

start_point = [10,10];
start_pCharger = [10,10;10,10;10,10;10,10;10,10;10,10;10,10];
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
userConfig.popSize=400*3*2*2; % population size
userConfig.numIter = 600*2;     % number of max iteration
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
userConfig.alpha_ratio(:,1) = .2/3*15;

userConfig.alpha_ratio(1:300,5) = (1:300)*.0013;
userConfig.alpha_ratio(301:600,5) = (300:-1:1)*.0012;
userConfig.alpha_ratio(601:900,5) = (1:300)*.0013;
userConfig.alpha_ratio(901:end,5) = .4;
% userConfig.alpha_ratio(901:1200,5) = (300:-1:1)*.0012;
% userConfig.alpha_ratio(1201:1500,5) = (1:300)*.0013;
% userConfig.alpha_ratio(1501:end,5) = .5;
% userConfig.alpha_ratio(:,5) = 0.5;
% weight vector: the first one is for travel distance the fifth one is for chargers energy

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
% Evaluate of the trajectories and get the timeline and plot
[traj_segment,time_given_charger] = preplan_plot_oil(a,userConfig.numTarChargers,userConfig,start_point,start_pCharger);
disp(a.charger_travel_time)
a.time_given_charger
a.minTime

% Monte Carlo Simulation

monte_iter = 100;
aM = cell(1,monte_iter);

parfor i = 1:monte_iter
    a = ga4_0(userConfig);
    aM{i} = a;
end
save('file_name')
Run_time = toc/60