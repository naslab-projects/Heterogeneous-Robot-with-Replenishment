function varargout = stage1_auv_planner(varargin)

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
nth_worker_record = cell(popSize,1); % lifan added
charger_location_ind_record = cell(popSize,1); % lifan added

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
        
        nth_worker_charge = []; % new add by lifan
        
        % start evaluating for each salesman
        for s = 1:nSalesmen
            %d = start_mat(pRoute(rng(s,1)),(1-1)*2+1); % add starting point
            d = start_mat(pRoute(rng(s,1)),(s-1)*2+1);
            %time = d/start_mat(pRoute(rng(s,1)),(1-1)*2+2); % add time start point
            time = d/start_mat(pRoute(rng(s,1)),(s-1)*2+2);  % divide by worker speed 
            time_charging = time;          
            for k = rng(s,1):rng(s,2)-1  % loop for each battery life; record first
                time_charging = time_charging + dmat(pRoute(k),pRoute(k+1))/delta_v;
                if time_charging > pBatteryLife(s,man_battery_count(s)+1) % if the battery is enough to make this travel
                    time_start_charging(battery_life_count) = time; % time to start charging
                    time = time + charging_time;
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
        indexStations{p} = index_Stations;
        
        %% add charging period of time to the shorest travel time robots
        % find how many workers need to deploy late
        if nSalesmen > numTarChargers
            numLateDeploy = nSalesmen - numTarChargers;
            % sort the mission time
            [~,idx_late] = sort(mission_time,2);
            idx_late = idx_late(1:numLateDeploy);
            % change time start and finish charging
            temp = cumsum(man_battery_count);
            
            bi = [[1 temp(1:end-1)+1]' temp'];
            for il = 1:numLateDeploy
                
                time_start_charging(bi(idx_late(il),1):bi(idx_late(il),2)) = time_start_charging(bi(idx_late(il),1):bi(idx_late(il),2)) + charging_time;
                time_end_charging(bi(idx_late(il),1):bi(idx_late(il),2)) = time_end_charging(bi(idx_late(il),1):bi(idx_late(il),2)) + charging_time;
            end
            mission_time(idx_late) = mission_time(idx_late) + charging_time;
        end
        
        cell_mission_time{p} = mission_time;
        
        %% Charging time constraint
        
        [scheduled_start, scheduled_ind] = sort(time_start_charging); % sort time schedule
        pStation = pRoute(index_Stations);
        scheduled_charger = pStation(scheduled_ind);  % real position
        % scheduled_start = time_start_charging(scheduled_ind);
        scheduled_end = time_end_charging(scheduled_ind);
        scheduled_nth_worker_charge  = nth_worker_charge (scheduled_ind); % add by lifan from begining to the end
        % 
               
        %% calculate the cost
        num_chargering_period = length(scheduled_charger);
        NUm = ones(1,numTarChargers)*floor(num_chargering_period/numTarChargers);
        remainder_NUm = rem(num_chargering_period,numTarChargers);
        NUm(1:remainder_NUm) = NUm(1:remainder_NUm) + 1; % find the number of charging period for each charger
        
        ic_count = 1;
        new_order_pStation = zeros(1,num_chargering_period);
        time_start = zeros(1,num_chargering_period);
        time_end = zeros(1,num_chargering_period);
        
        new_nth_worker_charge = zeros(1,num_chargering_period); % add by lifan 
        % scheduling for chargers
        for ic = 1:numTarChargers
            for jc = 1:NUm(ic)
                new_order_pStation(ic_count) = scheduled_charger((jc-1)*numTarChargers+ic);
                new_nth_worker_charge(ic_count) = scheduled_nth_worker_charge((jc-1)*numTarChargers+ic);% add by lifan 
                time_start(ic_count) = scheduled_start((jc-1)*numTarChargers+ic);
                time_end(ic_count) = scheduled_end((jc-1)*numTarChargers+ic);
                ic_count = ic_count + 1;
            end
        end
        charger_location{p} = new_order_pStation;
        nth_worker_record{p} = new_nth_worker_charge; % add by lifan 
        charging_time_start{p} = time_start;  % add by lifan 
        %% Calculate charger energy
        time_charger = zeros(numTarChargers,40);
        time_charger_travel = zeros(1,length(new_order_pStation)-numTarChargers+1);
        time_end_start = zeros(1,length(new_order_pStation)-numTarChargers+1);
        cum_NUm = cumsum(NUm);
        charger_location_ind = [[1,cum_NUm(1:end-1)+1];cum_NUm]';
        charger_count = 1;
        
        for ic = 1:numTarChargers
            charger_energy_count = 1;
            % time_charger_travel(charger_count) = start_mat(new_order_pStation(charger_location_ind(ic,1)),2*1+ic)/delta_vc;
            time_charger_travel(charger_count) = start_mat(new_order_pStation(charger_location_ind(ic,1)),2*configStruct.nSalesmen+ic)/delta_vc;  % lifan add 2023_12_27
            time_end_start(charger_count) = time_start(charger_location_ind(ic,1));
            charger_count = charger_count + 1;
            charger_energy_count = charger_energy_count + 1;
            for jc = charger_location_ind(ic,1):charger_location_ind(ic,2)-1
                time_charger_travel(charger_count) = dmat(new_order_pStation(jc),new_order_pStation(jc+1))/delta_vc; % add here charger distance estimation
                time_charger(ic,charger_energy_count) = time_charger_travel(charger_count);
                time_end_start(charger_count) = time_start(jc+1) - time_end(jc);
                charger_count = charger_count + 1;
                charger_energy_count = charger_energy_count + 1;
            end
        end
        time_travel_charger{p} = time_charger_travel;
        time_given_charger{p} = time_end_start;
        total_ds(p) = sum(sum(time_charger));
        charger_location_ind_record{p} = charger_location_ind; % lifan added
        
        %% Calculate the speed limit constraint
        %         average_charging = mean(time_end_start);
        %         speed_limit_penalty(p) = alpha_ratio(4)*(sum(abs(time_end_start - sum(time_end_start)/length(time_end_start)))+ sum(abs(time_charger_travel - sum(time_charger_travel)/length(time_charger_travel)))); % vectorize to improve the speed
        %         speed_limit_penalty(p) = sum(abs(time_charger_travel - sum(time_charger_travel)/length(time_charger_travel))); % vectorize to improve the speed
        for ic = 1:length(time_charger_travel)
            if time_charger_travel(ic) > time_end_start(ic)
                speed_limit_penalty(p) = speed_limit_penalty(p) + time_charger_travel(ic) - time_end_start(ic);
            end
        end
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
        
        %% Use fitness function
        totalDist(p) = sum(total_distance); % overall distance
        
        % time used from preplan
%         mission_time = mission_time + pp_time;
        totalTime(p) = max(mission_time); % mission time
        energy_cost(p) = sum(mission_time);
        
        % normalize charger distance

        % fitness function
        fun(p) = alpha_ratio(iter,1)*energy_cost(p) + alpha_ratio(iter,5)*speed_limit_penalty(p); 
%         + alpha_ratio(iter,4)*mean(time_charger_travel); 
        
        % add penalty for constraints
%         fun(p) = fun(p) + alpha_ratio(iter,5)*speed_limit_penalty(p);

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
        min_nth_worker_record = nth_worker_record{index}; % lifan added
        min_charger_location_ind_record = charger_location_ind_record{index};
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
            title(hAx,sprintf('Travel Distance = %1.2f, ds = %1.2f, N_w = %d, N_c = %d,Dpenalty = %1.2f',minDist,minds,nSalesmen,numStations(index),d_penalty ));
            hold(hAx,'off');
            drawnow;
        end
%     end
    trackDist(iter) = minDist;
    trackDs(iter) = minds;
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
                    
%                 case 5 % clip
%                     if I+5 > length(rtes) | J+5 > length(rtes)
%                         tmpPopRoute(k,[I:J]) = tmpPopRoute(k,[I:J]); % keep the same
%                     else
%                         tmp_22 = tmpPopRoute(k,[I:J]);
%                         tmpPopRoute(k,[I:J]) = tmpPopRoute(k,[I+5:J+5]);
%                         tmpPopRoute(k,[I+5:J+5]) = tmp_22;
%                     end          
%                     tmpPopBatteryLife{k} = bestOf8BatteryLife;
%                     %                     tmpindexStations{k} = bestOf8Index;                    
                    
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
        'nth_worker_record', min_nth_worker_record, ... % lifan added nth UAV [2 1 3 1 2 3]
        'charger_location_ind_record', min_charger_location_ind_record, ... % lifan added nth ASV  charger decide color # 1
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

