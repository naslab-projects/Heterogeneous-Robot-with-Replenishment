% AUV1 = [traj_segment([1,2]) a.minTime(1)-2*charging_period-sum(traj_segment([1,2]))];
% AUV2 = [traj_segment([3,4]) a.minTime(2)-3*charging_period-sum(traj_segment([3,4]))];
% AUV3 = [traj_segment([5,6]) a.minTime(3)-2*charging_period-sum(traj_segment([5,6]))];
clr = [1 0 0; 0 0 1; 1 0 1; 0 1 0; 1 0.5 0; .5 .5 0 ;0 0 0; 1 .5 .5];


% last_battery1 = a.minTime(1)-2*charging_period-sum(traj_segment([1,2]));
% last_battery2 = a.minTime(2)-3*charging_period-sum(traj_segment([3,4]));
% last_battery3 = a.minTime(3)-2*charging_period-sum(traj_segment([5,6]));

last_battery(1) = a.minTime(1)-2*charging_period-sum(traj_segment([1,2]));
last_battery(2) = a.minTime(2)-2*charging_period-sum(traj_segment([3,4]));
last_battery(3) = a.minTime(3)-2*charging_period-sum(traj_segment([5,6]));

[longest_time, wait_uav_idx] = max(a.minTime);  % remove a waiting time for the longest operation uav
last_battery(wait_uav_idx) = last_battery(wait_uav_idx) - charging_period;


% Find the indices of negative elements
% a.charger_travel_time % real travel time  usually small
% a.time_given_charger % time limit         usually big
interval = a.time_given_charger - a.charger_travel_time; % should be all positve. if not, the auv should wait
negative_indx = find(interval < 0);

if length(negative_indx)>0   % 1 2 3 (asv11)   4 5 6(asv2)
    disp('please insert waiting time')
end

auv_delay6 = a.charger_travel_time(6) - a.time_given_charger(6);
 
auv_delay = a.charger_travel_time(wait_uav_idx) - a.time_given_charger(wait_uav_idx);
if auv_delay < 0
    auv_delay = 0;
end
    
if auv_delay6 < 0
    auv_delay6 = 0;
end

% auv2_delay = a.charger_travel_time(2) - a.time_given_charger(2);


% AUV1 = [traj_segment(1) charging_period                            traj_segment(2) charging_period last_battery(1)];
% AUV2 = [traj_segment(3) charging_period+auv2_delay charging_period traj_segment(4) charging_period last_battery(2)];
% AUV3 = [traj_segment(5) charging_period                            traj_segment(6) charging_period last_battery(3)];

AUV3 = [traj_segment(5) charging_period+auv_delay   charging_period  traj_segment(6) auv_delay6 charging_period last_battery(3)];
AUV2 = [traj_segment(3) charging_period                              traj_segment(4) charging_period last_battery(2)];
AUV1 = [traj_segment(1) charging_period                              traj_segment(2) charging_period last_battery(1)];



len_auv1 = length(AUV1) ;% 5
len_auv2 = length(AUV2); % 5
len_auv3 = length(AUV3); % 6


%% save time

save('auv_time.mat', 'AUV1', 'AUV2', 'AUV3');   

%%


figure

c = categorical({'AUV1','AUV2','AUV3'}); 

% schedule_tolerance = [0.0420    0.0767    0.2    0.2    0.2    0.0497]'; % auto
% rest_delay = 0.2-schedule_tolerance;

% schedule_y = [traj_segment',schedule_tolerance,rest_delay];
% schedule_y = zeros(9,1);
% schedule_y([1,2,4,5,7,8]) = traj_segment(1:6)';
% schedule_y([3,6,9]) = last_battery_life_time';

schedule_y = zeros(3,len_auv1+len_auv2+len_auv3);
% schedule_y(1,1:5) = AUV1;
% schedule_y(2,6:10) = AUV2;
% schedule_y(3,11:end) = AUV3;


schedule_y(1,1:len_auv1) = AUV1;
schedule_y(2,len_auv1+1:len_auv1+len_auv2) = AUV2;
schedule_y(3,len_auv1+len_auv2+1:end) = AUV3;



% schedule_bar = barh(c,schedule_y,'stacked');

% clr = [1 0 0;1 0 0; 1 0 0; 0 0 1;0 0 1; 0 0 1;1 0 1;1 0 1;1 0 1];
% schedule_bar(1).FaceColor = 'flat';
% % schedule_bar(2).FaceColor = 'y';
% % schedule_bar(3).FaceColor = 'g';
% for i = 1:9
%     schedule_bar(1).CData(i,:) = clr(i,:);
% end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Define colors for each segment
% colors_AUV3 = [1 0 1; 0.9290 0.6940 0.1250; 1 0 1; 0 1 0; 1 0 1];
% colors_AUV2 = [0 0 1; 0 0 0; 0 1 0; 0 0 1; 0.9290 0.6940 0.1250; 0 0 1;];
% colors_AUV1 = [1 0 0; 0 1 0; 1 0 0; 0.9290 0.6940 0.1250; 1 0 0];


red__ = [1 0 0];
green = [0 1 0];
black = [0 0 0];
orang = [0.9290 0.6940 0.1250];
blue_ = [0 0 1];
purpl = [1 0 1]


% colors_AUV3 = [purpl; black; green; purpl; black; orang; purpl];
% colors_AUV2 = [blue_; orang; blue_; green; blue_];
% colors_AUV1 = [red__; green; red__; orang; red__];


colors_AUV3 = [purpl; black; green; purpl; black; orang; purpl];
colors_AUV2 = [blue_; green; blue_; orang; blue_];
colors_AUV1 = [red__; orang; red__; green; red__];


colors = [colors_AUV1; colors_AUV2; colors_AUV3];

% Plot stacked bar with specified colors
schedule_bar = barh(c, schedule_y, 'stacked','FaceColor','flat');
% for i = 1:size(schedule_y, 1) % 3
%     for j = 1:size(schedule_y, 2) % 16
%         schedule_bar(j).FaceColor = colors(i, :);
%     end
% end


% bar (X)  CData(Y)  

for i = 1:size(colors_AUV1, 1)
    schedule_bar(i).CData(1,:) = colors_AUV1(i, :);
end

for i = 1:size(colors_AUV2, 1) %6~11
    %schedule_bar(5+i).CData(2,:) = colors_AUV2(i, :);
    schedule_bar(len_auv1+i).CData(2,:) = colors_AUV2(i, :);
end

for i = 1:size(colors_AUV3, 1) %12
    %schedule_bar(11+i).CData(3,:) = colors_AUV3(i, :);
    schedule_bar(len_auv1+len_auv2+i).CData(3,:) = colors_AUV3(i, :);
end



% Add legend with color labels
% legend({'Segment 1','Segment 2','Segment 3','Segment 4','Segment 5','Segment 6'}, 'Location', 'SouthOutside', 'Orientation', 'horizontal');
% Adjust color labels on the y-axis
yticklabels({'AUV1','AUV2','AUV3'});





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% legend('boxoff')
% legend('AUV#1','AUV#2','AUV#3','charger#1','charger#2','waiting')

xlabel('Time (h)')