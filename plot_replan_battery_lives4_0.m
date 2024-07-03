last_battery1 = 4/3;
last_battery2 = 2/3;

auv2_delay = a.charger_travel_time(2) - a.time_given_charger(2);

AUV1 = [traj_segment(1) charging_period traj_segment1(1) charging_period traj_segment1(2) charging_period last_battery1];
AUV2 = [traj_segment(3) charging_period+auv2_delay charging_period traj_segment1(3) charging_period  traj_segment1(4) charging_period last_battery2];
AUV3 = [traj_segment(5) charging_period];
figure

c = categorical({'AUV1','AUV2','AUV3'}); 

% schedule_tolerance = [0.0420    0.0767    0.2    0.2    0.2    0.0497]'; % auto
% rest_delay = 0.2-schedule_tolerance;

% schedule_y = [traj_segment',schedule_tolerance,rest_delay];
% schedule_y = zeros(9,1);
% schedule_y([1,2,4,5,7,8]) = traj_segment(1:6)';
% schedule_y([3,6,9]) = last_battery_life_time';

schedule_y = zeros(3,17);
schedule_y(1,1:length(AUV1)) = AUV1;
schedule_y(2,length(AUV1)+1:length(AUV1)+length(AUV2)) = AUV2;
schedule_y(3,length(AUV1)+length(AUV2)+1:end) = AUV3;
schedule_bar = barh(c,schedule_y,'stacked');

% clr = [1 0 0;1 0 0; 1 0 0; 0 0 1;0 0 1; 0 0 1;1 0 1;1 0 1;1 0 1];
% schedule_bar(1).FaceColor = 'flat';
% % schedule_bar(2).FaceColor = 'y';
% % schedule_bar(3).FaceColor = 'g';
% for i = 1:9
%     schedule_bar(1).CData(i,:) = clr(i,:);
% end
legend('AUV#1','AUV#2','AUV#3','charger#1','charger#2','waiting')
% legend('boxoff')
xlabel('Time (h)')