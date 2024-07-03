% mission specific parameters
who_traj = [1,3,6];
who_charg = [1,3];
who_fail = 2;
numWorker = 2;
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
clr = [1 0 0; 0 0 1; 1 0 1;  1 0.5 0;0 1 0; .5 .5 0 ;0 0 0; 1 .5 .5];
clr = [clr;zeros(10,3)];
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
start_pCharger1 = a.xy(a.optRoute(first_rend(who_charg)),:); % charger
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
get_battery_level = [userConfig.batteryLife userConfig.batteryLife];

userConfig.nSalesmen = numWorker;
userConfig.start_mat = start_mat1;
userConfig.initial_pop = repmat(old_solution,userConfig.popSize,1);
userConfig.initial_pop = sort(userConfig.initial_pop,2);
userConfig.break1 = break2;
userConfig.initial_battery_level = get_battery_level;
% run GA solver 
userConfig.alpha_ratio(:,4) = 0.05;
userConfig.pp_time = Mission_time_current(who_charg);
a2 = replan_position_ga(userConfig);
%% plot replan
replan_plot(a2,userConfig.numTarChargers,userConfig,start_point1,start_pCharger1)
disp(a2.time_given_charger)
disp(a2.charger_travel_time)

for i = 1:a.nSalesmen
    temp_first = [temp_first temp_break(i):first_rend(i)];
    coverNum(i) = length(temp_break(i):first_rend(i));
    rte = a.optRoute(temp_break(i):first_rend(i));
    plot([start_point(i,1); a.xy(rte,1)],[start_point(i,2); a.xy(rte,2)],'.-','linewidth',2,'Color',clr(i,:));
    hold('on');
end
% C = char('s','o');
% for i = 1:numCharger
%      plot(start_pCharger(i,1),start_pCharger(i,2),C(i),'linewidth',2,'MarkerSize',20,'MarkerEdgeColor',clr(numWorker+i,:),'MarkerFaceColor',clr(numWorker+i,:))
%      text(start_pCharger(i,1)-.2,start_pCharger(i,2),'1','color','w','FontSize',14)
% end
i = 2;
plot(3,11,'o','linewidth',2,'MarkerSize',20,'MarkerEdgeColor',clr(numWorker+i,:),'MarkerFaceColor',clr(numWorker+i,:))
text(3-.2,11,'1','color','w','FontSize',14)
i= 3;
plot(9,6,'s','linewidth',2,'MarkerSize',20,'MarkerEdgeColor',clr(numWorker+i,:),'MarkerFaceColor',clr(numWorker+i,:))
text(9-.2,6,'1','color','w','FontSize',14)
 
legend('AUV#1','AUV#2','AUV#3','charger#1','charger#2','border')
legend('boxoff')
% %% plot final
% final_plot(a2,userConfig.numTarChargers,userConfig,start_point1,start_pCharger1)
% for i = 1:a.nSalesmen
%     temp_first = [temp_first temp_break(i):first_rend(i)];
%     coverNum(i) = length(temp_break(i):first_rend(i));
%     rte = a.optRoute(temp_break(i):first_rend(i));
%     plot([start_point(i,1); a.xy(rte,1)],[start_point(i,2); a.xy(rte,2)],'.-','linewidth',2,'Color',clr(i,:));
%     hold('on');
% end
% 
% for i = 1:numCharger
%      plot(start_pCharger(i,1),start_pCharger(i,2),C(i),'linewidth',2,'MarkerSize',20,'MarkerEdgeColor',clr(numWorker+i,:),'MarkerFaceColor',clr(numWorker+i,:))
%      text(start_pCharger(i,1)-.2,start_pCharger(i,2),'1','color','w','FontSize',14)
% end
% legend('AUV#1','AUV#2','AUV#3','charger#1','charger#2','border')
% legend('boxoff')
plot(start_pCharger(1,1),start_pCharger(1,2),'k*','linewidth',2,'MarkerSize',20)