[traj_segment,time_given_charger] = preplan_plot_oil(a,userConfig.numTarChargers,userConfig,start_point,start_pCharger);

last_battery1 = a.minTime(1)-3*charging_period-sum(traj_segment([1,2]));
last_battery2 = a.minTime(2)-3*charging_period-sum(traj_segment([3,4]));
last_battery3 = a.minTime(3)-3*charging_period-sum(traj_segment([5,6,7]));
last_battery4 = a.minTime(4)-3*charging_period-sum(traj_segment([8,9,10]));
last_battery5 = a.minTime(5)-3*charging_period-sum(traj_segment([11,12]));
last_battery6 = a.minTime(6)-3*charging_period-sum(traj_segment([13,14,15]));
last_battery7 = a.minTime(7)-4*charging_period-sum(traj_segment([16,17,18,19]));

% auv2_delay = a.charger_travel_time(2) - a.time_given_charger(2);
auv1_delay = 2.3;

AUV1 = [traj_segment(1) charging_period charging_period traj_segment(2) auv1_delay charging_period last_battery1];
AUV2 = [traj_segment(3) charging_period charging_period traj_segment(4) charging_period last_battery2];
AUV3 = [traj_segment(5) charging_period traj_segment(6) charging_period traj_segment(7) charging_period last_battery3];
AUV4 = [traj_segment(8) charging_period traj_segment(9) charging_period traj_segment(10) charging_period last_battery4];
AUV5 = [traj_segment(11) charging_period charging_period traj_segment(12) charging_period last_battery5];
AUV6 = [traj_segment(13) charging_period traj_segment(14) charging_period traj_segment(15) charging_period last_battery6];
AUV7 = [traj_segment(16) charging_period traj_segment(17) charging_period traj_segment(18) charging_period traj_segment(19) charging_period last_battery7];

figure

c = categorical({'AUV1','AUV2','AUV3','AUV4','AUV5','AUV6','AUV7'}); 

% schedule_tolerance = [0.0420    0.0767    0.2    0.2    0.2    0.0497]'; % auto
% rest_delay = 0.2-schedule_tolerance;

% schedule_y = [traj_segment',schedule_tolerance,rest_delay];
% schedule_y = zeros(9,1);
% schedule_y([1,2,4,5,7,8]) = traj_segment(1:6)';
% schedule_y([3,6,9]) = last_battery_life_time';
total_bars = sum([length(AUV1),length(AUV2),length(AUV3),length(AUV4),length(AUV5),length(AUV6),length(AUV7)]);
cum_bars = cumsum([length(AUV1),length(AUV2),length(AUV3),length(AUV4),length(AUV5),length(AUV6),length(AUV7)]);

schedule_y = zeros(7,total_bars);

schedule_y(1,1:length(AUV1)) = AUV1;
schedule_y(2,cum_bars(1)+1:cum_bars(2)) = AUV2;
schedule_y(3,cum_bars(2)+1:cum_bars(3)) = AUV3;
schedule_y(4,cum_bars(3)+1:cum_bars(4)) = AUV4;
schedule_y(5,cum_bars(4)+1:cum_bars(5)) = AUV5;
schedule_y(6,cum_bars(5)+1:cum_bars(6)) = AUV6;
schedule_y(7,cum_bars(6)+1:end) = AUV7;

schedule_bar = barh(c,schedule_y,'stacked');

% clr = [1 0 0;1 0 0; 1 0 0; 0 0 1;0 0 1; 0 0 1;1 0 1;1 0 1;1 0 1];
% schedule_bar(1).FaceColor = 'flat';
% % schedule_bar(2).FaceColor = 'y';
% % schedule_bar(3).FaceColor = 'g';
% for i = 1:9
%     schedule_bar(1).CData(i,:) = clr(i,:);
% end
xlabel('Time (h)')