function varargout = ga4_3_drone_CPP(varargin)

% Initialize default configuration
defaultConfig.xy          = 10*rand(40,2);
defaultConfig.dmat        = [];
defaultConfig.nSalesmen   = 5;
defaultConfig.minTour     = 2;
defaultConfig.popSize     = 80;
defaultConfig.numIter     = 5e3*4;
defaultConfig.showProg    = true;
defaultConfig.showResult  = true;
defaultConfig.showWaitbar = true;
defaultConfig.batteryLife = 5;
defaultConfig.initial_battery_level = 1;  
defaultConfig.alpha_ratio = ones(1500,5);
defaultConfig.start_end_point = 1;
defaultConfig.numTarChargers = 2;
defaultConfig.step_size = 0.0001;
defaultConfig.priority = ones(10);
defaultConfig.charging_window = 0.3;
defaultConfig.delta_v = ones(207)*1;
defaultConfig.delta_vc = ones(207)*1;
defaultConfig.start_mat = ones(50,2);
defaultConfig.charging_time= 15;
defaultConfig.initial_pop = [];
defaultConfig.break1 = [];
defaultConfig.pp_time = [];
% Interpret user configuration inputs
if ~nargin
    userConfig = struct();
elseif isstruct(varargin{1})
    userConfig = varargin{1};
else
    try
        userConfig = struct(varargin{:});
    catch
        error('Expected inputs are either a structure or parameter/value pairs');
    end
end

% Override default configuration with user inputs
configStruct = get_config(defaultConfig,userConfig);

% Extract configuration
xy          = configStruct.xy;
dmat        = configStruct.dmat;
nSalesmen   = configStruct.nSalesmen;
minTour     = configStruct.minTour;
popSize     = configStruct.popSize;
numIter     = configStruct.numIter;
showProg    = configStruct.showProg;
showResult  = configStruct.showResult;
showWaitbar = configStruct.showWaitbar;
temp_batteryLife = configStruct.batteryLife;
initial_battery_level = configStruct.initial_battery_level;
alpha_ratio = configStruct.alpha_ratio;
delta_v     = configStruct.delta_v;
delta_vc    = configStruct.delta_vc;
start_mat   = configStruct.start_mat;
% step_size   = configStruct.step_size;
charging_time = configStruct.charging_time;
% start_end_point = configStruct.start_end_point; %(TODO: include start and end point for mission)
numTarChargers = configStruct.numTarChargers;
pp_time = configStruct.pp_time;
% priority = configStruct.priority;
charging_window = configStruct.charging_window;




if isempty(dmat)
    nPoints = size(xy,1);
    a = meshgrid(1:nPoints);
    dmat = reshape(sqrt(sum((xy(a,: )-xy(a',:)).^2,2)),nPoints,nPoints);
end
initial_pop = configStruct.initial_pop;
break1 = configStruct.break1;
% Verify Inputs
[N,dims] = size(xy);
[nr,nc] = size(dmat);
if N ~= nr || N ~= nc
    error('Invalid XY or DMAT inputs!')
end
% n = N;
n = size(initial_pop,2);
% Sanity Checks
nSalesmen   = max(1,min(n,round(real(nSalesmen(1)))));
minTour     = max(1,min(floor(n/nSalesmen),round(real(minTour(1)))));
popSize     = max(8,8*ceil(popSize(1)/8));
numIter     = max(1,round(real(numIter(1))));
showProg    = logical(showProg(1));
showResult  = logical(showResult(1));
showWaitbar = logical(showWaitbar(1));

% Initializations for Route Break Point Selection
% nBreaks = nSalesmen-1;
% dof = n - minTour*nSalesmen;          % degrees of freedom
% addto = ones(1,dof+1);
% for k = 2:nBreaks
%     addto = cumsum(addto);
% end
%     cumProb = cumsum(addto)/sum(addto);

% Initialize the Populations
popRoute = initial_pop;         % population of routes
% popBreak = zeros(popSize,nBreaks);   % population of breaks
indexStations = cell(popSize,1);
popBatteryLife = cell(popSize,1);
battery_life_distance = cell(popSize,1);
battery_life_time = cell(popSize,1);
% priority_overall = cell(popSize,1);
mobile_charging = cell(popSize,1);
charger_location = cell(popSize,1);
time_travel_charger = cell(popSize,1);
charging_time_start = cell(popSize,1);
charging_time_end = cell(popSize,1);
time_given_charger = cell(popSize,1);

% popRoute(1,:) = (1:n);
% breakl = cumsum(ones(1,nSalesmen-1)*floor(n/nSalesmen));
optBreak = break1;
rng = [[1 break1+1];[break1 n]]';
BatteryLife_temp = ones(nSalesmen,20)*temp_batteryLife; % limitation for one worker is 20 battery life
BatteryLife_temp(:,1) = initial_battery_level;

popBatteryLife{1} = BatteryLife_temp;
% TODO: fix the initial population problem
for k = 2:popSize
%     popRoute(k,:) = n:-1:1;
    popBatteryLife{k} = BatteryLife_temp;
end
%     for k = 2:popSize
%         popRoute(k,:) = randperm(n);
%         popBreak(k,:) = rand_breaks();
%         [numStations(k), indexStations{k}] = find_stations(popRoute(k,:),popBreak(k,:));
%     end

% Select the Colors for the Plotted Routes
pclr = ~get(0,'DefaultAxesColor');
clr = [1 0 0; 0 0 1; 1 0 1; 0 1 0; 1 0.5 0];
if nSalesmen > 5
    clr = hsv(nSalesmen);
end

% Run the GA
globalMin = Inf;
total_ds = zeros(1,popSize);
totalTime = zeros(1,popSize);
energy_cost = zeros(1,popSize);
totalDist = zeros(1,popSize);
distHistory = zeros(1,numIter);
trackDist = zeros(1,numIter);
trackDs = zeros(1,numIter);
trackDp = zeros(1,numIter);
tmpPopRoute = zeros(8,n);
tmpPopBatteryLife = cell(1,8);
% tmpindexStations = cell(1,8);
newPopRoute = zeros(popSize,n);
newPopBatteryLife = cell(1,8);
cell_mission_time = cell(1,popSize);

if showProg
    figure('Name','MTSPO_GA | Current Best Solution','Numbertitle','off');
    hAx = gca;
end
if showWaitbar
    hWait = waitbar(0,'Searching for near-optimal solution ...');
end

for iter = 1:numIter
    % Evaluate Members of the Population
    fun = zeros(1,popSize);
    speed_limit_penalty =zeros(1,popSize);
    numStations = zeros(1,popSize);
        
    for p = 1:popSize
        % define the chromosome
        pRoute = popRoute(p,:);
        pBatteryLife = popBatteryLife{p};
        % clear and allocate memory
        total_distance = zeros(1,nSalesmen);
        mission_time = zeros(1,nSalesmen);
        
        time_start_charging = zeros(1,50); % TODO: give a more accurate estimate
        time_end_charging = zeros(1,50);
        index_Stations = zeros(1,50);
        
        count = 1;
        battery_life_count = 1;
        man_battery_count = zeros(1,nSalesmen);
        
        % start evaluating for each salesman  ,  drone distance "mission_time" will be minimized 
        % record which Salesman need to be charged
        nth_worker_charge = []; % new add by lifan
        record_uav_usv = []; % new add by lifan
        % record_uav_usv(end + 1, :) = [s, dist, arrive_time, uavpos, usvpos];
        usv_end_charging_time = [0 0 0 0]; % 1x4 usv1 usv2 > return_flight_time + charging_period 
        % uav_charge_order_time = [0 0 0];  % choose the smallest time to go 
        asv1_1 = [];  % asv charging schedule 
        asv1_2 = [];  % each asv has two charging spots
        asv2_1 = [];
        asv2_2 = [];
        uav_overlap_penalty = 0; 
%% lifan added, count by salesmen        
%         % uav1 uav2 uav3 waypoint_len
%         for s = 1:nSalesmen
%             waypoint_len(s) = rng(s,1) - rng(s,1) +1;
%         end
%         [M_max, I_max] = max(waypoint_len);
%         max_waypoint = M_max;  % if the UAV runs out of the point just skip it.
%                 
%         
%         % time(1) time(2) time(3) time (s) for UAVs
%         % time_charging(1) time_charging(2) time_charging(3) time_charging(s) for UAVs
%         for k = 1:max_waypoint            
%             % UAV1  UAV2  UAV3
%             for s = 1:nSalesmen
%                 if k == 1 % start_point
%                     d(s) = start_mat(pRoute(rng(s,1)),(s-1)*2+1);
%                     time(s) = d(s)/userConfig.uav_return_speed;% 60 km/ hr
%                     time_charging(s) = time(s);  
%                 else  % all other waypoints k > 1                                       
%                     % update k value
%                     k = rng(s,1) + k - 1  % because 1 + 2  -1 = 2     
%                     if k == rng(s,2)
%                        % already in the end , it is a protection 
%                     else
%                        % still not touch the end
%                        time_charging(s) = time_charging(s) + dmat(pRoute(k),pRoute(k+1))/delta_v; % is used for reset
%                        if time_charging(s) > pBatteryLife(s,man_battery_count(s)+1) % if the battery is enough to make this travel, skip this if                        
%                            % Flag Up,  how much travel time need to use, decide later to choose usv1 or usv2
%                            uav_charge_order_time(s) = time(s);  % lifan add
%                            
%                            time_start_charging(battery_life_count) = time(s); % time to start charging                 
%                            time(s) = time(s) + charging_time;                           
%                            time_end_charging(battery_life_count) = time_start_charging(battery_life_count) + charging_time; % time finished charging
%                            time_charging(s) = dmat(pRoute(k),pRoute(k+1))/delta_v;   % reset the  battery level
%                            index_Stations(battery_life_count) = count;
%                            battery_life_count = battery_life_count + 1;
%                            man_battery_count(s) = man_battery_count(s) + 1;
%                            % nth_worker_charge = [nth_worker_charge s];  % new add by lifan record which Salesman need to be charged
% 
%                        end
%                        d(s) = d(s) + dmat(pRoute(k),pRoute(k+1)); % distance
%                        time(s) = time(s) + dmat(pRoute(k),pRoute(k+1))/delta_v; % time for each battery life
%                        count = count + 1;
% 
%                     end
%                         
%                 end
%             end
%             % Compare flag,          
%             % Update usv_end_charging_time
%             for s = 1:nSalesmen
%                % update choose add auv1 time or auv 2 time  
%                
%             end
%         end

        
%         for s = 1:nSalesmen
%             %d = start_mat(pRoute(rng(s,1)),(1-1)*2+1); % add starting point
%             d = start_mat(pRoute(rng(s,1)),(s-1)*2+1);
%             %time = d/start_mat(pRoute(rng(s,1)),(1-1)*2+2); % add time start point
%             %time = d/start_mat(pRoute(rng(s,1)),(s-1)*2+2);  % divide by worker speed  8 km/h
%             time = d/userConfig.uav_return_speed;% 60 km/ hr
%             time_charging = time;   
%             for k = rng(s,1):rng(s,2)-1  % loop for each battery life; record first
%                 time_charging = time_charging + dmat(pRoute(k),pRoute(k+1))/delta_v;
%                 if time_charging > pBatteryLife(s,man_battery_count(s)+1) % if the battery is enough to make this travel, skip this if
%                     time_start_charging(battery_life_count) = time; % time to start charging                 
%                     time = time + charging_time;
%                     time_end_charging(battery_life_count) = time_start_charging(battery_life_count) + charging_time; % time finished charging
%                     time_charging = dmat(pRoute(k),pRoute(k+1))/delta_v;   % reset the  battery level
%                     index_Stations(battery_life_count) = count;
%                     battery_life_count = battery_life_count + 1;
%                     man_battery_count(s) = man_battery_count(s) + 1;
%                     nth_worker_charge = [nth_worker_charge s];  % new add by lifan record which Salesman need to be charged
%                 end
%                 d = d + dmat(pRoute(k),pRoute(k+1)); % distance
%                 time = time + dmat(pRoute(k),pRoute(k+1))/delta_v; % time for each battery life
%                 count = count + 1;
%             end
% %             d = d + dmat(pRoute(rng(s,2)),pRoute(rng(s,1))); % make it a circle
%             count = count + 1;
%             total_distance(s) = d; % distance for each salesman
%             mission_time(s) = time;
%         end
        % remove zeros
        
%         time_start_charging = time_start_charging(time_start_charging~=0);
%         time_end_charging = time_end_charging(time_end_charging~=0);
%         index_Stations = index_Stations(index_Stations~=0);
%         indexStations{p} = index_Stations;   % 1x6
        
        
%%        
        for s = 1:nSalesmen
            %d = start_mat(pRoute(rng(s,1)),(1-1)*2+1); % add starting point
            d = start_mat(pRoute(rng(s,1)),(s-1)*2+1);
            %time = d/start_mat(pRoute(rng(s,1)),(1-1)*2+2); % add time start point
            %time = d/start_mat(pRoute(rng(s,1)),(s-1)*2+2);  % divide by worker speed  8 km/h
            time = d/userConfig.uav_return_speed;% 60 km/ hr
            time_charging = time;   
            for k = rng(s,1):rng(s,2)-1  % loop for each battery life; record first
                time_charging = time_charging + dmat(pRoute(k),pRoute(k+1))/delta_v;
                if time_charging > pBatteryLife(s,man_battery_count(s)+1) % if the battery is enough to make this travel, skip this if
                    time_start_charging(battery_life_count) = time; % time to start charging
                                  
                    % s is the nth UAV
                    % position of UAV 
                    uavpos = pRoute(k); %count; % 1~196 uav_position
                    
                    % position of USVs
                    if time < 17  % 9+8  drone choose the first charging cycle
                        charger_idx = 1;
                    elseif time < 27  % 9+8+8 = 27 second charging cycle
                        charger_idx = 2;
                    else
                        charger_idx = 3;
                    end                    
                    % charger_position 1x4 , need to remove start point  1+ charger_idx
                    asv1_xy = userConfig.asv1_xy(1+charger_idx,:);
                    asv2_xy = userConfig.asv2_xy(1+charger_idx,:);
                    % transfer it into the idx map
                    % Check for rows in A that match vector B
                    indices_1 = ismember(xy, asv1_xy, 'rows'); %ismember(A, B, 'rows');
                    indices_2 = ismember(xy, asv2_xy, 'rows');
                    % Find the row indices where the match occurs 
                    asv1_row_indices = find(indices_1);
                    asv2_row_indices = find(indices_2);                   
                    
                    % Calculate distance between UAVs and USVs
                    dist_asv1 = dmat(uavpos,asv1_row_indices);
                    dist_asv2 = dmat(uavpos,asv2_row_indices);
                    
                    % Calculate flight time 
                    return_to_asv1_flight_time = dist_asv1/userConfig.uav_return_speed; % 60 km/ hr
                    return_to_asv2_flight_time = dist_asv2/userConfig.uav_return_speed; % 60 km/ hr
                    
                    % arrived_time
                    arrive_asv1_time = time + return_to_asv1_flight_time;
                    arrive_asv2_time = time + return_to_asv2_flight_time;
                    
                    % parking which asv
                    which_asv = 0;
 
                    %% charging schedule
                    if arrive_asv1_time < arrive_asv2_time
                        % mean the distance is smaller than asv2, first check asv1
                        %[updatedA, updatedB, space_remain] = insertIntervals(A, B, C);
                        C = [arrive_asv1_time arrive_asv1_time+userConfig.charging_time_drone];
                        [asv1_1, asv1_2, space_remain] = insertIntervals(asv1_1, asv1_2, C);
                        if space_remain == 0
                            % mean no space in the asv 1 so go to the asv 2
                            C = [arrive_asv2_time arrive_asv2_time+userConfig.charging_time_drone];
                            [asv2_1, asv2_2, space_remain] = insertIntervals(asv2_1, asv2_2, C);
                            if space_remain == 0
                                %disp('no space to park asv1 asv2')
                                uav_overlap_penalty = uav_overlap_penalty + 1;
                            elseif space_remain == 1 
                                nth_charging_spot = 3; % on asv 2
                            elseif space_remain == 2
                                nth_charging_spot = 4; % on asv 2  
                            end 
                            record_uav_usv(end + 1, :) = [s, dist_asv2, arrive_asv2_time, uavpos, asv2_row_indices, nth_charging_spot];
                            which_asv = 2;
                        else
                            if space_remain == 1 
                                nth_charging_spot = 1; % on asv 1
                            elseif space_remain == 2
                                nth_charging_spot = 2; % on asv 1
                            end
                            record_uav_usv(end + 1, :) = [s, dist_asv1, arrive_asv1_time, uavpos, asv1_row_indices, nth_charging_spot];
                            which_asv = 1;
                        end                       
                    else
                        % mean the distance is smaller than asv2, first check asv2
                        C = [arrive_asv2_time arrive_asv2_time+userConfig.charging_time_drone];
                        [asv2_1, asv2_2, space_remain] = insertIntervals(asv2_1, asv2_2, C);
                        if space_remain == 0
                            % mean no space in the asv 1 so go to the asv 1
                            C = [arrive_asv1_time arrive_asv1_time+userConfig.charging_time_drone];
                            [asv1_1, asv1_2, space_remain] = insertIntervals(asv1_1, asv1_2, C);
                            if space_remain == 0
                                %disp('no space to park asv1 asv2')
                                uav_overlap_penalty = uav_overlap_penalty + 1;
                            elseif space_remain == 1 
                                nth_charging_spot = 1; % on asv 1
                            elseif space_remain == 2
                                nth_charging_spot = 2; % on asv 1  
                            end   
                            record_uav_usv(end + 1, :) = [s, dist_asv1, arrive_asv1_time, uavpos, asv1_row_indices, nth_charging_spot];
                            which_asv = 1;
                        else
                            if space_remain == 1 
                                nth_charging_spot = 3; % on asv 2
                            elseif space_remain == 2
                                nth_charging_spot = 4; % on asv 2
                            end       
                            record_uav_usv(end + 1, :) = [s, dist_asv2, arrive_asv2_time, uavpos, asv2_row_indices, nth_charging_spot];
                            which_asv = 2;
                        end    
                    end
                    
                    if which_asv == 1
                        return_flight_time = return_to_asv1_flight_time*2;                        
                    elseif which_asv == 2
                        return_flight_time = return_to_asv2_flight_time*2;  
                    end
%%                   
                    % it does not work because the for loop will first
                    % consider all the path for one robot and then consider
                    % the next salesman, so usv_end_charging_time threshold
                    % does not work. Thus, we decide a schedule map to
                    % record
%                     if arrive_asv1_time < arrive_asv2_time
%                         % mean the distance is smaller than asv2, first check asv1
%                         if arrive_asv1_time > usv_end_charging_time(1)  % uav can charge on asv1 
%                             record_uav_usv(end + 1, :) = [s, dist_asv1, arrive_asv1_time, uavpos, asv1_row_indices]; 
%                             usv_end_charging_time(1) = arrive_asv1_time + userConfig.charging_time_drone; % update the end charging time
%                             
%                         elseif arrive_asv1_time > usv_end_charging_time(2)  % uav can charge on asv1
%                             record_uav_usv(end + 1, :) = [s, dist_asv1, arrive_asv1_time, uavpos, asv1_row_indices]; 
%                             usv_end_charging_time(2) = arrive_asv1_time + userConfig.charging_time_drone; % update the end charging time
%                             
%                         elseif arrive_asv1_time > usv_end_charging_time(3) % uav can charge on asv2
%                             record_uav_usv(end + 1, :) = [s, dist_asv2, arrive_asv2_time, uavpos, asv2_row_indices]; 
%                             usv_end_charging_time(3) = arrive_asv2_time + userConfig.charging_time_drone; % update the end charging time
%                             
%                         elseif arrive_asv1_time > usv_end_charging_time(4) % uav can charge on asv2 
%                             record_uav_usv(end + 1, :) = [s, dist_asv2, arrive_asv2_time, uavpos, asv2_row_indices];
%                             usv_end_charging_time(4) = arrive_asv2_time + userConfig.charging_time_drone; % update the end charging time
%                              
%                         else
%                             disp('error')
%                             pause;
%                         end
%                     else
%                         % mean the distance is smaller than asv1, first check asv2
%                         if arrive_asv2_time > usv_end_charging_time(3) % uav can charge on asv2
%                             record_uav_usv(end + 1, :) = [s, dist_asv2, arrive_asv2_time, uavpos, asv2_row_indices]; 
%                             usv_end_charging_time(3) = arrive_asv2_time + userConfig.charging_time_drone; % update the end charging time                                
%                             
%                         elseif arrive_asv2_time > usv_end_charging_time(4) % uav can charge on asv2 
%                             record_uav_usv(end + 1, :) = [s, dist_asv2, arrive_asv2_time, uavpos, asv2_row_indices];
%                             usv_end_charging_time(4) = arrive_asv2_time + userConfig.charging_time_drone; % update the end charging time                   
%                           
%                         elseif arrive_asv2_time > usv_end_charging_time(1)  % uav can charge on asv1
%                             record_uav_usv(end + 1, :) = [s, dist_asv1, arrive_asv1_time, uavpos, asv1_row_indices]; 
%                             usv_end_charging_time(1) = arrive_asv1_time + userConfig.charging_time_drone; % update the end charging time
%                             
%                         elseif arrive_asv2_time > usv_end_charging_time(2)  % uav can charge on asv1
%                             record_uav_usv(end + 1, :) = [s, dist_asv1, arrive_asv1_time, uavpos, asv1_row_indices]; 
%                             usv_end_charging_time(2) = arrive_asv1_time + userConfig.charging_time_drone; % update the end charging time   
%                             
%                         else
%                             disp('error')
%                             pause;
%                         end                                     
%                     end
%%                    
                    %time = time + charging_time;
                    time = time + charging_time + return_flight_time; % lifan add 
                    time_end_charging(battery_life_count) = time_start_charging(battery_life_count) + charging_time; % time finished charging
                    time_charging = dmat(pRoute(k),pRoute(k+1))/delta_v;   % reset the  battery level
                    index_Stations(battery_life_count) = count;
                    battery_life_count = battery_life_count + 1;
                    man_battery_count(s) = man_battery_count(s) + 1;
                    nth_worker_charge = [nth_worker_charge s];  % new add by lifan record which Salesman need to be charged
                    
                end
                d = d + dmat(pRoute(k),pRoute(k+1)); % distance
                time = time + dmat(pRoute(k),pRoute(k+1))/delta_v; % time for each battery life
                count = count + 1;
            end
%             d = d + dmat(pRoute(rng(s,2)),pRoute(rng(s,1))); % make it a circle
            count = count + 1;
            total_distance(s) = d; % distance for each salesman
            mission_time(s) = time;
        end
        % remove zeros
        time_start_charging = time_start_charging(time_start_charging~=0);
        time_end_charging = time_end_charging(time_end_charging~=0);
        index_Stations = index_Stations(index_Stations~=0);
        indexStations{p} = index_Stations;   % 1x6
        
%%        
        % add by lifan 
        % record_uav_usv(end + 1, :) = [s, dist, arrive_time, uavpos, usvpos];
        charging_arrange(p).data = record_uav_usv; % nx1 record all charging arrangement in each gene 800 
        total_uavs_return_distance =  sum(record_uav_usv(:, 2));% should time*2,  sum_second_column
        total_ds(p) = total_uavs_return_distance;        
        uav_overlap_penalty_record(p) = uav_overlap_penalty;
        %% calculate the distance between the drone (UAV 1 2 3) and the boat (ASV 1 2)
%         if time_start_charging % 1x6
%         userConfig.asy1_xy
%         userConfig.asy2_xy

%         [scheduled_start, scheduled_ind] = sort(time_start_charging); % sort time schedule % nth_worker_charge
%         pStation = pRoute(index_Stations);  % 1 1 2 2 3 3  position
%         scheduled_charger = pStation(scheduled_ind);  % the position when workers need to be charged from the beginning to the end
%  
%         %cycle_charging = length(time_start_charging)/ nSalesmen;% 6/3 = 2
%         %len= length(time_start_charging)
%         pStation              % position
%         time_start_charging   % time for charging
%         nth_worker_charge     % worker 1~3
      
%         thresholdValue = userConfig.charging_time_drone;   %2.0;  % Adjust this threshold as needed
%         charging_order = proximity(time_start_charging, thresholdValue)
%         cycle_charging = max(charging_order);  % max charging cycle
%         
%         % cycle_charging = floor(length(time_start_charging)/ nSalesmen) %6/3 = 2 not good because some drone will stay more than twice
%         % boat position changes with time
%         
%         record_dist_charging = zeros(cycle_charging,2+nSalesmen+2);  %  distance, charging pattern, involved worker1, worker2, worker3, asv1 asv2 2x7
%         
%         
%         for cycle = 1:cycle_charging   %1~2 ~ ... nth 
%             
%             
%             %Check current chargering position, according to time
%             if time_start_charging(cycle) < 17  % 9+8  drone choose the first charging cycle
%                 charger_idx = 1;
%             elseif time_start_charging(cycle) < 27  % 9+8+8 = 27 second charging cycle
%                 charger_idx = 2;
%             else
%                 charger_idx = 3;
%             end
%             % charger_position 1x4 , need to remove start point  1+ charger_idx
%             asv1_xy = userConfig.asv1_xy(1+charger_idx,:);
%             asv2_xy = userConfig.asv2_xy(1+charger_idx,:);
%             
%             % transfer it into the idx map
%             % Check for rows in A that match vector B
%             indices_1 = ismember(xy, asv1_xy, 'rows'); %ismember(A, B, 'rows');
%             indices_2 = ismember(xy, asv2_xy, 'rows');
%             % Find the row indices where the match occurs
%             asv1_row_indices = find(indices_1);
%             asv2_row_indices = find(indices_2);
%             %%  calculate how many robots want to land in this cycle, by proximity function 
%             
%             num_robot = sum(charging_order == cycle);
%             if num_robot == 3
%                 % calculate distantce between drone and boat, use dmat(A,B)    
%                 numberToFind = cycle;  % find the same cycle robot
%                 indices = find(charging_order == numberToFind); % 3x1
%                 involved_pStation = pStation(indices); % poistion 1x3
%                 worker1 = involved_pStation(1);
%                 worker2 = involved_pStation(2);
%                 worker3 = involved_pStation(3);
%                 % Initialize the array  distance 
%                 dist_uav_asv = zeros(nSalesmen, 2); % 3 robots x 2 [asv1 asv2] 
%                 for uav_idx = 1:nSalesmen  %1~3
%                     %dist_asv1 = dmat(pStation(cycle + cycle_charging*(uav_idx-1)),asv1_row_indices);
%                     %dist_asv2 = dmat(pStation(cycle + cycle_charging*(uav_idx-1)),asv2_row_indices);
%                     dist_asv1 = dmat(involved_pStation(uav_idx),asv1_row_indices);
%                     dist_asv2 = dmat(involved_pStation(uav_idx),asv2_row_indices);
%                     dist_uav_asv(uav_idx, :) = [dist_asv1 dist_asv2];
%                 end
%                 % six charging pattern
%                 % 1 1 2
%                 % 1 2 1
%                 % 1 2 2
%                 % 2 2 1
%                 % 2 1 1
%                 % 2 1 2
% 
%                 % 1 1 2  -> 1
%                 arrange_uav(1) = dist_uav_asv(1,1) + dist_uav_asv(2,1) + dist_uav_asv(3,2);
%                 % 1 2 1  -> 2
%                 arrange_uav(2) = dist_uav_asv(1,1) + dist_uav_asv(2,2) + dist_uav_asv(3,1);
%                 % 1 2 2  -> 3
%                 arrange_uav(3) = dist_uav_asv(1,1) + dist_uav_asv(2,2) + dist_uav_asv(3,2);
%                 % 2 2 1  -> 4
%                 arrange_uav(4) = dist_uav_asv(1,2) + dist_uav_asv(2,2) + dist_uav_asv(3,1);
%                 % 2 1 1  -> 5
%                 arrange_uav(5) = dist_uav_asv(1,2) + dist_uav_asv(2,1) + dist_uav_asv(3,1);
%                 % 2 1 2  -> 6
%                 arrange_uav(6) = dist_uav_asv(1,2) + dist_uav_asv(2,1) + dist_uav_asv(3,2);
%                 % where M - is the min value
%                 % and I - is index of the minimum value
%                 [M_min,I_min] = min(arrange_uav);
%                 % compare and choose the minize distance and memorize path
%                 record_dist_charging(cycle,:) = [M_min I_min worker1 worker2 worker3 asv1_row_indices asv2_row_indices]; %  distance, charging pattern, involved nth robots
%             elseif num_robot == 2
%                 numberToFind = cycle;  % find the same cycle robot
%                 indices = find(charging_order == numberToFind); % 2x1
%                 involved_robots = nth_worker_charge(indices); % robots 1x2
%                 involved_pStation = pStation(indices); % poistion 1x2
%                 
%                 [worker1, worker2, worker3] = deal(0, 0, 0); % record involved robots number
%                 dist_2 = zeros(num_robot, 2); % 2 robots x 2 [asv1 asv2]  % Initialize the array  distance 
%                 for nth_idx = 1:2
%                     nth_robot_idx = involved_robots(nth_idx);  % 1 or 2 or 3
%                     if nth_robot_idx == 1
%                         worker1 = involved_pStation(nth_idx);
%                     elseif nth_robot_idx == 2
%                         worker2 = involved_pStation(nth_idx);
%                     elseif nth_robot_idx == 3
%                         worker3 = involved_pStation(nth_idx);
%                     end
%                     dist_asv1 = dmat(involved_pStation(nth_idx),asv1_row_indices);
%                     dist_asv2 = dmat(involved_pStation(nth_idx),asv2_row_indices);
%                     dist_2(nth_idx, :) = [dist_asv1 dist_asv2];
%                 end
%                 % four charging pattern
%                 % 1 1
%                 % 1 2
%                 % 2 1
%                 % 2 2
%                 
%                 % 1 1 -> 1
%                 arr_uav(1) = dist_2(1,1) + dist_2(2,1);
%                 % 1 2 -> 2
%                 arr_uav(2) = dist_2(1,1) + dist_2(2,2);
%                 % 2 1 -> 3
%                 arr_uav(3) = dist_2(1,2) + dist_2(2,1);
%                 % 2 2 -> 4    
%                 arr_uav(4) = dist_2(1,2) + dist_2(2,2);
%                 [M_min,I_min] = min(arr_uav);
%                 record_dist_charging(cycle,:) = [M_min I_min worker1 worker2 worker3 asv1_row_indices asv2_row_indices]; %  distance, charging pattern, involved nth robots
%             else % num_robot == 1 
%                 numberToFind = cycle;  % find the same cycle robot
%                 indice = find(charging_order == numberToFind); % 1x1
%                 involved_robot = nth_worker_charge(indice); % robots 1x1
%                 involved_pStation = pStation(indice); % poistion 1x1
%                 
%                 [worker1, worker2, worker3] = deal(0, 0, 0); % record involved robots number
%                 nth_robot_idx = involved_robot;  % 1 or 2 or 3
%                 if nth_robot_idx == 1
%                     worker1 = involved_pStation;
%                 elseif nth_robot_idx == 2
%                     worker2 = involved_pStation;
%                 elseif nth_robot_idx == 3
%                     worker3 = involved_pStation;
%                 end
%                 dist_1 = zeros(1, 2); % 1 robots x 2 [asv1 asv2] 
%                 dist_asv1 = dmat(involved_pStation, asv1_row_indices);        
%                 dist_asv2 = dmat(involved_pStation, asv2_row_indices); 
%                 dist_1 = [dist_asv1 dist_asv2];
%                 % charging pattern, land asv1 and land asv2
%                 [M_min,I_min] = min(dist_1);
%                 record_dist_charging(cycle,:) = [M_min I_min worker1 worker2 worker3 asv1_row_indices asv2_row_indices]; %  distance, charging pattern, involved nth robots
%             end
%             
%         end
%         
%         charging_arrange(p).data = record_dist_charging; % 2x1 record all charging arrangement in each gene 800 
%         total_dist_charging =  sum(record_dist_charging(:, 1));% should time*2,  sum_first_column
%         total_ds(p) = total_dist_charging;
        
        
        %% add charging period of time to the shorest travel time robots
        % find how many workers need to deploy late
%         if nSalesmen > numTarChargers
%             numLateDeploy = nSalesmen - numTarChargers;
%             % sort the mission time
%             [~,idx_late] = sort(mission_time,2);
%             idx_late = idx_late(1:numLateDeploy);
%             % change time start and finish charging
%             temp = cumsum(man_battery_count);
%             
%             bi = [[1 temp(1:end-1)+1]' temp'];
%             for il = 1:numLateDeploy
%                 
%                 time_start_charging(bi(idx_late(il),1):bi(idx_late(il),2)) = time_start_charging(bi(idx_late(il),1):bi(idx_late(il),2)) + charging_time;
%                 time_end_charging(bi(idx_late(il),1):bi(idx_late(il),2)) = time_end_charging(bi(idx_late(il),1):bi(idx_late(il),2)) + charging_time;
%             end
%             mission_time(idx_late) = mission_time(idx_late) + charging_time;
%         end
%         cell_mission_time{p} = mission_time;
        
        %% Charging time constraint
        
%         [scheduled_start, scheduled_ind] = sort(time_start_charging); % sort time schedule
%         pStation = pRoute(index_Stations);
%         scheduled_charger = pStation(scheduled_ind);  % the position when workers need to be charged from the beginning to the end
%         % scheduled_start = time_start_charging(scheduled_ind);
%         scheduled_end = time_end_charging(scheduled_ind);
        
        %% calculate the cost
%         num_chargering_period = length(scheduled_charger);
%         NUm = ones(1,numTarChargers)*floor(num_chargering_period/numTarChargers);
%         remainder_NUm = rem(num_chargering_period,numTarChargers);
%         NUm(1:remainder_NUm) = NUm(1:remainder_NUm) + 1; % find the number of charging period for each charger
%         
%         ic_count = 1;
%         new_order_pStation = zeros(1,num_chargering_period);
%         time_start = zeros(1,num_chargering_period);
%         time_end = zeros(1,num_chargering_period);
%         % scheduling for chargers
%         for ic = 1:numTarChargers
%             for jc = 1:NUm(ic)
%                 new_order_pStation(ic_count) = scheduled_charger((jc-1)*numTarChargers+ic);
%                 time_start(ic_count) = scheduled_start((jc-1)*numTarChargers+ic);
%                 time_end(ic_count) = scheduled_end((jc-1)*numTarChargers+ic);
%                 ic_count = ic_count + 1;
%             end
%         end
%         charger_location{p} = new_order_pStation;
        
        
        %% Calculate charger energy
        
%         time_charger = zeros(numTarChargers,40);
%         time_charger_travel = zeros(1,length(new_order_pStation)-numTarChargers+1);
%         time_end_start = zeros(1,length(new_order_pStation)-numTarChargers+1);
%         cum_NUm = cumsum(NUm);
%         charger_location_ind = [[1,cum_NUm(1:end-1)+1];cum_NUm]';
%         charger_count = 1;
%         
%         for ic = 1:numTarChargers
%             charger_energy_count = 1;
%             %time_charger_travel(charger_count) = start_mat(new_order_pStation(charger_location_ind(ic,1)),2*1+ic)/delta_vc;
%             time_charger_travel(charger_count) = start_mat(new_order_pStation(charger_location_ind(ic,1)),2*configStruct.nSalesmen+ic)/delta_vc;
%             time_end_start(charger_count) = time_start(charger_location_ind(ic,1));
%             charger_count = charger_count + 1;
%             charger_energy_count = charger_energy_count + 1;
%             for jc = charger_location_ind(ic,1):charger_location_ind(ic,2)-1
%                 time_charger_travel(charger_count) = dmat(new_order_pStation(jc),new_order_pStation(jc+1))/delta_vc; % add here charger distance estimation
%                 time_charger(ic,charger_energy_count) = time_charger_travel(charger_count);
%                 time_end_start(charger_count) = time_start(jc+1) - time_end(jc);
%                 charger_count = charger_count + 1;
%                 charger_energy_count = charger_energy_count + 1;
%             end
%         end
%         time_travel_charger{p} = time_charger_travel;
%         time_given_charger{p} = time_end_start;
%         total_ds(p) = sum(sum(time_charger));
        
        %% Calculate the speed limit constraint
        %         average_charging = mean(time_end_start);
        %         speed_limit_penalty(p) = alpha_ratio(4)*(sum(abs(time_end_start - sum(time_end_start)/length(time_end_start)))+ sum(abs(time_charger_travel - sum(time_charger_travel)/length(time_charger_travel)))); % vectorize to improve the speed
        %         speed_limit_penalty(p) = sum(abs(time_charger_travel - sum(time_charger_travel)/length(time_charger_travel))); % vectorize to improve the speed
%         for ic = 1:length(time_charger_travel)
%             if time_charger_travel(ic) > time_end_start(ic)  % = time_charger_travel(ic) > time_given_charger (ic) , if boat cannot arrive, mean that distance is too far, so need a punishment
%                 speed_limit_penalty(p) = speed_limit_penalty(p) + time_charger_travel(ic) - time_end_start(ic);
%             end
%         end
        % charging period constraint
%         charging_period_constraint(p) = sum(abs(time_end_start - sum(time_end_start)/length(time_end_start)));
        %         for ipp = 1:length(time_end_start)
        %             speed_limit_penalty(p) = speed_limit_penalty(p) + alpha_ratio(4)*abs(time_end_start(ipp)-sum(time_end_start)/length(time_end_start));
%         end
%         for ipp = 1: length(time_end_start)
%             if time_end_start(ipp) < time_charger_travel(ipp)
%                 speed_limit_penalty(p) = speed_limit_penalty(p) + alpha_ratio(4)*(time_charger_travel(ipp)- time_end_start(ipp));
%             end
%         end
        %% Calculate the distance between the point where the drone runs out of battery and the point where the boat stays
        
        
        
        



        %% Use fitness function
        totalDist(p) = sum(total_distance); % overall distance
        
        % time used from preplan
        %mission_time = mission_time + pp_time;
        totalTime(p) = max(mission_time); % mission time
        energy_cost(p) = sum(mission_time);
        
        % normalize charger distance

        % fitness function
        %fun(p) = alpha_ratio(iter,1)*energy_cost(p) + alpha_ratio(iter,5)*speed_limit_penalty(p); 
        fun(p) = alpha_ratio(iter,1)*energy_cost(p) + 0.1*total_ds(p) + uav_overlap_penalty_record(p);
        
        %+ alpha_ratio(iter,4)*mean(time_charger_travel); 
        
        % add penalty for constraints
        %fun(p) = fun(p) + alpha_ratio(iter,5)*speed_limit_penalty(p);

    end

    %% Selection
    %Find the Best Route in the Population
    [minFun,index] = min(fun);
    distHistory(iter) = minFun;
    
%     if minFun < globalMin
        globalMin = minFun;
        optRoute = popRoute(index,:);
        %optBreak = popBreak(index,:);
        ptemp = indexStations{index};
        optStations = optRoute(ptemp);
        minDist = totalDist(index);
        minTime = totalTime(index);
        find_mission_time = cell_mission_time{index};
        minEnergy = energy_cost(index);
        d_penalty = speed_limit_penalty(index);
        min_battery_life_distance = battery_life_distance{index};
        min_delta_t = battery_life_time{index};
        min_priority = [];
        min_mobile_charging = mobile_charging{index};
        min_charging_level = popBatteryLife{index};
        % TODO: min_ds is not valid
        minds = total_ds(index);
        min_charger_location = charger_location{index};
        min_time_charger_travel = time_travel_charger{index};
        min_time_given_charger = time_given_charger{index};
        min_charging_time_start = charging_time_start{index};
        min_charging_time_end = charging_time_end{index};
        min_charging_arrangement_uav =  charging_arrange(index);% lifan add
        fprintf('uav_overlap_penalty_record ( %d ) is %d\n', index, uav_overlap_penalty_record(index)); % lifan add , should be zero
        
        rng = [[1 optBreak+1];[optBreak n]]';
        if showProg
            % Plot the Best Route
            for s = 1:nSalesmen
                rte = optRoute(rng(s,1):rng(s,2));
                plot(hAx,xy(rte,1),xy(rte,2),'.-','linewidth',2,'Color',clr(s,:));
                hold(hAx,'on');
            end
            plot(xy(optStations,1),xy(optStations,2),'kx', 'linewidth', 2,'MarkerSize',10,'MarkerFaceColor','k')
            xlabel('X(km)')
            ylabel('Y(km)')
            %title(hAx,sprintf('Travel Distance = %1.2f, ds = %1.2f, N_w = %d, N_c = %d,Dpenalty = %1.2f',minDist,minds,nSalesmen,numStations(index),d_penalty ));
            title(hAx,sprintf('Travel Distance = %1.2f, Charging Distance = %1.2f',minDist,minds));
            hold(hAx,'off');
            drawnow;
        end
%     end
    trackDist(iter) = minDist;
    trackDs(iter) = minds;
    d_penalty = 0;  % lifan add
    trackDp(iter) = d_penalty;
    
    %% Genetic Algorithm Operators
    randomOrder = randperm(popSize);
    
    % change the crossover
    for p = 8:8:popSize
        rtes = popRoute(randomOrder(p-7:p),:);
        bles = popBatteryLife(randomOrder(p-7:p));
        %brks = popBreak(randomOrder(p-7:p),:);
        %sson = indexStations(randomOrder(p-7:p));
        dists = fun(randomOrder(p-7:p));
     
        [~,idx] = min(dists);
        bestOf8Route = rtes(idx,:);
        %bestOf8Break = brks(idx,:);
        %bestOf8Index = sson{idx};
        bestOf8BatteryLife = bles{idx};
        routeInsertionPoints = sort(ceil(n*rand(1,2)));
        I = routeInsertionPoints(1);
        J = routeInsertionPoints(2);
        
        for k = 1:8 % Generate New Solutions
            tmpPopRoute(k,:) = bestOf8Route;
            %             tmpPopBatteryLife{k} = bestOf8BatteryLife;
            %             tmpPopBreak(k,:) = bestOf8Break;
            
            switch k
                case 2 % Flip
                    tmpPopRoute(k,I:J) = tmpPopRoute(k,J:-1:I);
                    tmpPopBatteryLife{k} = bestOf8BatteryLife;
                    %                     tmpindexStations{k} = bestOf8Index;
                case 3 % Swap
                    tmpPopRoute(k,[I J]) = tmpPopRoute(k,[J I]);
                    tmpPopBatteryLife{k} = bestOf8BatteryLife;
                    %                     tmpindexStations{k} = bestOf8Index;
                case 4 % Slide
                    tmpPopRoute(k,I:J) = tmpPopRoute(k,[I+1:J I]);
                    tmpPopBatteryLife{k} = bestOf8BatteryLife;
                    %                     tmpindexStations{k} = bestOf8Index;
                case 5 % Modify threshold
                    some_temp = temp_batteryLife*(1 - charging_window*rand(nSalesmen,20));
%                     some_temp(:,1) = (1:nSalesmen)*floor(temp_batteryLife/nSalesmen);
                    some_temp(:,1) = initial_battery_level;
                    tmpPopBatteryLife{k} = some_temp;
                case 6 % Flip, Modify threshold
                    some_temp = temp_batteryLife*(1 - charging_window*rand(nSalesmen,20));
%                     some_temp(:,1) = (1:nSalesmen)*floor(temp_batteryLife/nSalesmen);
                    some_temp(:,1) = initial_battery_level;
                    tmpPopRoute(k,I:J) = tmpPopRoute(k,J:-1:I);
                    tmpPopBatteryLife{k} = some_temp;
                case 7 % Swap, Modify threshold
                    some_temp = temp_batteryLife*(1 - charging_window*rand(nSalesmen,20));
%                     some_temp(:,1) = (1:nSalesmen)*floor(temp_batteryLife/nSalesmen);
                    some_temp(:,1) = initial_battery_level;
                    tmpPopRoute(k,[I J]) = tmpPopRoute(k,[J I]);
                    tmpPopBatteryLife{k} = some_temp;
                case 8 % Slide, Modify thershold
                    some_temp = temp_batteryLife*(1 - charging_window*rand(nSalesmen,20));
%                     some_temp(:,1) = (1:nSalesmen)*floor(temp_batteryLife/nSalesmen);
                    some_temp(:,1) = initial_battery_level;
                    tmpPopRoute(k,I:J) = tmpPopRoute(k,[I+1:J I]);
                    tmpPopBatteryLife{k} = some_temp;
                otherwise % do nothing
                    tmpPopBatteryLife{k} = bestOf8BatteryLife;
            end
        end
        newPopRoute(p-7:p,:) = tmpPopRoute;
        newPopBatteryLife(p-7:p) = tmpPopBatteryLife;

    end
    popRoute = newPopRoute;
    popBatteryLife = newPopBatteryLife;

    % Update the waitbar
    if showWaitbar && ~mod(iter,ceil(numIter/325))
        waitbar(iter/numIter,hWait);
    end
    
end
if showWaitbar
    close(hWait);
end

if showResult
    % Plots
    figure('Name','MTSPO_GA | Results','Numbertitle','off');
    subplot(2,2,1);
    if dims > 2, plot3(xy(:,1),xy(:,2),xy(:,3),'.','Color',pclr);
    else plot(xy(:,1),xy(:,2),'.','Color',pclr); end
    title('City Locations');
    subplot(2,2,2);
    imagesc(dmat(optRoute,optRoute));
    title('Distance Matrix');
    subplot(2,2,3);
    rng = [[1 optBreak+1];[optBreak n]]';
    for s = 1:nSalesmen
        rte = optRoute(rng(s,1):rng(s,2));
        if dims > 2, plot3(xy(rte,1),xy(rte,2),xy(rte,3),'.-','Color',clr(s,:));
        else plot(xy(rte,1),xy(rte,2),'.-','Color',clr(s,:)); end
        title(sprintf('Total Distance = %1.4f',minDist));
        hold on;
    end
    subplot(2,2,4);
    plot(distHistory,'b','LineWidth',2);
    title('Best Solution History');
%     set(gca,'XLim',[0 numIter+1],'YLim',[0 1.1*max([1 distHistory])]);
end

% Return Output
if nargout
    resultStruct = struct( ...
        'xy',          xy, ...
        'dmat',        dmat, ...
        'nSalesmen',   nSalesmen, ...
        'minTour',     minTour, ...
        'popSize',     popSize, ...
        'numIter',     numIter, ...
        'showProg',    showProg, ...
        'showResult',  showResult, ...
        'showWaitbar', showWaitbar, ...
        'optRoute',    optRoute, ...
        'optBreak',    optBreak, ...
        'optStations',    optStations, ...
        'minTime',     find_mission_time, ...
        'minEnergy',   minEnergy, ...
        'minDist',     minDist, ...
        'globalMin',    globalMin, ...
        'minds',        minds, ...
        'trackDist',    trackDist, ...
        'trackDs',      trackDs, ...
        'trackDp',      trackDp, ...
        'delta_t',      min_delta_t, ...
        'priority_over_time',     min_priority, ...
        'charging_period', min_mobile_charging, ...
        'charging_location', min_charger_location, ...
        'charger_travel_time', min_time_charger_travel, ...
        'charging_period_start', min_charging_time_start, ...
        'charging_period_end', min_charging_time_end, ...
        'time_given_charger', min_time_given_charger, ...
        'charging_level', min_charging_level, ...
        'charging_arrangement_uav', min_charging_arrangement_uav, ... % lifan added
        'battery_life_distance',   min_battery_life_distance);
    varargout = {resultStruct};
end
end

% Subfunction to override the default configuration with user inputs
function config = get_config(defaultConfig,userConfig)

% Initialize the configuration structure as the default
config = defaultConfig;

% Extract the field names of the default configuration structure
defaultFields = fieldnames(defaultConfig);

% Extract the field names of the user configuration structure
userFields = fieldnames(userConfig);
nUserFields = length(userFields);

% Override any default configuration fields with user values
for i = 1:nUserFields
    userField = userFields{i};
    isField = strcmpi(defaultFields,userField);
    if nnz(isField) == 1
        thisField = defaultFields{isField};
        config.(thisField) = userConfig.(userField);
    end
end

end


% proximity
function B = proximity(A, threshold)
    groups = containers.Map('KeyType', 'double', 'ValueType', 'any');
    groupCount = 1;

    for i = 1:length(A)
        assigned = false;

        keys = groups.keys;
        for j = 1:length(keys)
            group = groups(keys{j});
            if any(abs(A(i) - group) <= threshold)
                groups(keys{j}) = [group, A(i)];
                assigned = true;
                B(i) = keys{j};  % Assign the group number to the current element
                break;
            end
        end

        if ~assigned
            groups(groupCount) = A(i);
            B(i) = groupCount;  % Assign the new group number to the current element
            groupCount = groupCount + 1;
        end
    end
end


function [updatedA, updatedB, space_remain] = insertIntervals(A, B, intervals)
    % Sort intervals based on start times
    intervals = sortrows(intervals, 1);

    % Initialize updated matrices
    updatedA = A;
    updatedB = B;
    space_remain = 8; % default is 8 
    % Iterate through intervals and insert into the appropriate matrix
    for i = 1:size(intervals, 1)
        startTime = intervals(i, 1);
        endTime = intervals(i, 2);

        % Determine which matrix to insert into
        if isempty(updatedA) || ~isIntervalOccupied(updatedA, startTime, endTime)
            % Insert into matrix A
            updatedA = insertIntoMatrix(updatedA, [startTime endTime]);
            space_remain = 1;
        elseif isempty(updatedB) || ~isIntervalOccupied(updatedB, startTime, endTime)
            % Insert into matrix B
            updatedB = insertIntoMatrix(updatedB, [startTime endTime]);
            space_remain = 2;
        else
            space_remain = 0;
            %error('Cannot insert interval, both matrices are occupied during the specified time.');
        end
    end
end


%% schedule planning

function occupied = isIntervalOccupied(matrix, startTime, endTime)
    % Check if the interval is occupied in the given matrix
    occupied = any(startTime >= matrix(:, 1) & startTime <= matrix(:, 2)) || ...
               any(endTime >= matrix(:, 1) & endTime <= matrix(:, 2));
end

function updatedMatrix = insertIntoMatrix(matrix, interval)
    % Find the appropriate position to insert the interval
    inserted = false;
    for j = 1:size(matrix, 1)
        if interval(1) < matrix(j, 1)
            matrix = [matrix(1:j-1, :); interval; matrix(j:end, :)];
            inserted = true;
            break;
        end
    end

    % If the interval is greater than all existing end times, append it
    if ~inserted
        matrix = [matrix; interval];
    end

    updatedMatrix = matrix;
end
