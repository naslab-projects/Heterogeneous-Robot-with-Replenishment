% mission specific parameters
who_traj = [1,3,5,8,11,13,16];
who_charg = [3,2];
who_fail = 5;
numWorker = 6;
% Trajectories left of each AUV for replan
first_rend = zeros(1,length(a.optStations));
for i = 1:length(a.optStations)
    first_rend(i) = find(a.optRoute == a.optStations(i));
end
first_rend = sort(first_rend);
% first trajectory of 2rd AUV
first_rend = first_rend(who_traj);

% original breaks 
temp_break = [1 a.optBreak+1];
temp_first = [];
% figure
clr = hsv(12);
clr(4,:) = [0.39,0.01,0.01];
clr(6,:) = [0.65,0.41,0.05];

% how many mission points are covered already for each AUV
for i = 1:a.nSalesmen
    temp_first = [temp_first temp_break(i):first_rend(i)];
    coverNum(i) = length(temp_break(i):first_rend(i));
end
% new break 
% how many points left
how_left = size(userConfig.xy,1) - length(temp_first);
break2 = cumsum(ones(1,numWorker-1)*floor(how_left/numWorker));

old_solution = a.optRoute;

% remove the covered points
old_solution(temp_first) = [];

%initial positions 
start_point1 = a.xy(a.optRoute(first_rend),:); % worker
start_point1(who_fail,:) = [];
% start_pCharger1 = a.xy(a.optRoute(first_rend(end-numCharger+1:end)),:); % charger
% start_pCharger1 = a.xy(a.optRoute(first_rend(who_charg)),:); % charger
start_pCharger1 = [15,35;50,35;70,40;55,20]/5; % charger
start_mat1 = zeros(length(map.mission_location),numWorker*2+numCharger);

% Calculate distance to start point
for start_i = 1:numWorker
    for is = 1: length(map.mission_location)
        start_mat1(is,(start_i-1)*2+1) = norm([map.mission_location(is,:)-start_point1(start_i,:)]);
        start_mat1(is,(start_i-1)*2+2) = v;
    end
end

% add charger initial positions
for start_i = 1: numCharger
    for is = 1: length(map.mission_location)
        start_mat1(is,2*numWorker+start_i) = norm(map.mission_location(is,:)-start_pCharger1(start_i,:));
    end
    crte = a.optRoute(first_rend(1+start_i));

end
% update inputs with the existing solution

% userConfig.xy(temp_first,:) = [];

% battery level
get_battery_level = [userConfig.batteryLife userConfig.batteryLife userConfig.batteryLife userConfig.batteryLife userConfig.batteryLife userConfig.batteryLife];

userConfig.nSalesmen = numWorker;
userConfig.start_mat = start_mat1;
userConfig.initial_pop = repmat(old_solution,userConfig.popSize,1);
userConfig.initial_pop = sort(userConfig.initial_pop,2);
userConfig.break1 = break2;
userConfig.initial_battery_level = get_battery_level;
% run GA solver 
userConfig.alpha_ratio(:,4) = 0.05;
% userConfig.pp_time = Mission_time_current(who_charg);
a2 = replan_position_ga4_0(userConfig);
%% plot replan
traj_segment1 = replan_plot_oil4_0(a2,userConfig.numTarChargers,userConfig,start_point1,start_pCharger);
% disp(a2.time_given_charger)
% disp(a2.charger_travel_time)

% Plot chargers

% charger_location_ind = [4 4;1,2];
% st1 = [3,2;7,11];
% st2 = [3,10;6,12];
% st1 = [3,12;9,4];
% st2 = [7,10;6,12];
% 
% for ic = 1:numCharger 
%   plot(st1(ic,:),st2(ic,:),':','LineWidth',2,'Color',clr(numWorker+1+ic,:))
% end
% 
% legend('border','start')
% legend('boxoff')
for i = 1:a.nSalesmen
    temp_first = [temp_first temp_break(i):first_rend(i)];
    coverNum(i) = length(temp_break(i):first_rend(i));
    rte = a.optRoute(temp_break(i):first_rend(i));
    plot([start_point(1,1); a.xy(rte,1)]*scala,[start_point(1,2); a.xy(rte,2)]*scala,'.-','linewidth',2,'Color',clr(i,:));
    hold('on');
end
% 
% 
% for ic = 1:numCharger
%     
%     plot([start_point(1,1); a.xy(a.charging_location(charger_location_ind(ic,1):charger_location_ind(ic,2)),1)], ...
%         [start_point(1,2); a.xy(a.charging_location(charger_location_ind(ic,1):charger_location_ind(ic,2)),2)],':d','LineWidth',2,'MarkerSize',15,'Color',clr(numWorker+1+ic,:))
% end
