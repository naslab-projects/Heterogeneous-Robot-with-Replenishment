% mission specific parameters
who_traj = [1 3 5 8 11 13];
who_charg = [3,2];
who_fail = [1,2];
numWorker = 4;
% Trajectories left of each AUV for replan
first_rend1 = zeros(1,length(a2.optStations));
for i = 1:length(a2.optStations)
    first_rend1(i) = find(a2.optRoute == a2.optStations(i));
end
first_rend1 = sort(first_rend1);
% first trajectory of 2rd AUV
first_rend1 = first_rend1(who_traj);

% original breaks 
temp_break1 = [1 a2.optBreak+1];
temp_first1 = [];
% figure
clr = hsv(12);
clr(4,:) = [0.39,0.01,0.01];
clr(6,:) = [0.65,0.41,0.05];

% how many mission points are covered already for each AUV
for i = 1:a2.nSalesmen
    temp_first1 = [temp_first1 temp_break1(i):first_rend1(i)];
    coverNum(i) = length(temp_break1(i):first_rend1(i));
end
% new break 
% how many points left
how_left = length(a2.optRoute) - length(temp_first1) ;
break3 = cumsum(ones(1,numWorker-1)*floor(how_left/numWorker));

old_solution = a2.optRoute;

% remove the covered points
old_solution(temp_first1) = [];

%initial positions 
start_point1 = a2.xy(a2.optRoute(first_rend1),:); % worker
start_point1(who_fail,:) = [];
% start_pCharger1 = a.xy(a.optRoute(first_rend(end-numCharger+1:end)),:); % charger
% start_pCharger1 = a.xy(a.optRoute(first_rend(who_charg)),:); % charger
start_pCharger2 = [20,20;50,35;90,30;35,25]/5; % charger
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
        start_mat1(is,2*numWorker+start_i) = norm(map.mission_location(is,:)-start_pCharger2(start_i,:));
    end
    crte = a2.optRoute(first_rend1(1+start_i));

end
% update inputs with the existing solution

% userConfig.xy(temp_first,:) = [];

% battery level
get_battery_level = [userConfig.batteryLife userConfig.batteryLife userConfig.batteryLife userConfig.batteryLife];

userConfig.nSalesmen = numWorker;
userConfig.start_mat = start_mat1;
userConfig.initial_pop = repmat(old_solution,userConfig.popSize,1);
userConfig.initial_pop = sort(userConfig.initial_pop,2);
userConfig.break1 = break3;
userConfig.initial_battery_level = get_battery_level;
% run GA solver 
userConfig.alpha_ratio(:,4) = 0.05;
% userConfig.pp_time = Mission_time_current(who_charg);
a3 = replan_position_ga4_0(userConfig);
%% plot replan
traj_segment2 = replan_plot_oil4_0(a3,userConfig.numTarChargers,userConfig,start_point1,start_pCharger);
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
clr = hsv(12);
clr(4,:) = [0.39,0.01,0.01];
clr(6,:) = [0.65,0.41,0.05];
for i = 1:a.nSalesmen
    temp_first = [temp_first temp_break(i):first_rend(i)];
    coverNum(i) = length(temp_break(i):first_rend(i));
    rte = a.optRoute(temp_break(i):first_rend(i));
    plot([start_point(1,1); a.xy(rte,1)]*scala,[start_point(1,2); a.xy(rte,2)]*scala,'.-','linewidth',2,'Color',clr(i,:));
    hold('on');
end
clr = hsv(12);
clr(4,:) = [0.39,0.01,0.01];
clr(6,:) = [0.65,0.41,0.05];
clr = [clr(1:4,:);clr(6:end,:)];

start_pointW = [35,30;30,40;20,50;45,30;60,30;75,45]/5;

for i = 1:a2.nSalesmen
    temp_first1 = [temp_first1 temp_break1(i):first_rend1(i)];
    coverNum(i) = length(temp_break1(i):first_rend1(i));
    rte = a2.optRoute(temp_break1(i):first_rend1(i));
    plot([start_pointW(i,1); a2.xy(rte,1)]*scala,[start_pointW(i,2); a2.xy(rte,2)]*scala,'.-','linewidth',2,'Color',clr(i,:));
    hold('on');
end

sp1 = [50,50;75,45;15,35;20,20];
sp2 = [50,50;60,30;35,30;50,35];
sp3 = [50,50;45,30;30,40;70,40;90,30];
sp4 = [50,50;20,50;35,25];

% for i = 1:numCharger
%     
%     plot([start_point(1,1);start_pCharger1(i,1) ;start_pCharger2(i,1)]*scala, ...
%         [start_point(1,2);start_pCharger1(i,2) ;start_pCharger2(i,2)]*scala,':d','LineWidth',2,'MarkerSize',15,'Color',clr(a2.nSalesmen+i,:))
% end
plot( sp1(:,1),sp1(:,2),':d','LineWidth',2,'MarkerSize',15,'Color',clr(a2.nSalesmen+1,:))
plot( sp2(:,1),sp2(:,2),':d','LineWidth',2,'MarkerSize',15,'Color',clr(a2.nSalesmen+2,:))
plot( sp3(:,1),sp3(:,2),':d','LineWidth',2,'MarkerSize',15,'Color',clr(a2.nSalesmen+3,:))
plot( sp4(:,1),sp4(:,2),':d','LineWidth',2,'MarkerSize',15,'Color',clr(a2.nSalesmen+4,:))

st1 = [20,20;15,15];
st2 = [50,35;45,15];
st3 = [90,30;75,25];
st4 = [35,25;75,15];

plot( st1(:,1),st1(:,2),':','LineWidth',2,'Color',clr(a2.nSalesmen+1,:))
plot( st2(:,1),st2(:,2),':','LineWidth',2,'Color',clr(a2.nSalesmen+2,:))
plot( st3(:,1),st3(:,2),':','LineWidth',2,'Color',clr(a2.nSalesmen+3,:))
plot( st4(:,1),st4(:,2),':','LineWidth',2,'Color',clr(a2.nSalesmen+4,:))
% 
% for ic = 1:numCharger
%     
%     plot([start_point(1,1); a.xy(a.charging_location(charger_location_ind(ic,1):charger_location_ind(ic,2)),1)], ...
%         [start_point(1,2); a.xy(a.charging_location(charger_location_ind(ic,1):charger_location_ind(ic,2)),2)],':d','LineWidth',2,'MarkerSize',15,'Color',clr(numWorker+1+ic,:))
% end
