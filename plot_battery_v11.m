% AUV1 = [traj_segment([1,2]) a.minTime(1)-2*charging_period-sum(traj_segment([1,2]))];
% AUV2 = [traj_segment([3,4]) a.minTime(2)-3*charging_period-sum(traj_segment([3,4]))];
% AUV3 = [traj_segment([5,6]) a.minTime(3)-2*charging_period-sum(traj_segment([5,6]))];
clr = [1 0 0; 0 0 1; 1 0 1; 0 1 0; 1 0.5 0; .5 .5 0 ;0 0 0; 1 .5 .5];


% last_battery1 = a.minTime(1)-2*charging_period-sum(traj_segment([1,2]));
% last_battery2 = a.minTime(2)-3*charging_period-sum(traj_segment([3,4]));
% last_battery3 = a.minTime(3)-2*charging_period-sum(traj_segment([5,6]));

% last_battery(1) = a.minTime(1)-2*charging_period-sum(traj_segment([1,2]));
% last_battery(2) = a.minTime(2)-2*charging_period-sum(traj_segment([3,4]));
% last_battery(3) = a.minTime(3)-2*charging_period-sum(traj_segment([5,6]));

% five segement,  segement number = charger_times,  

% last_battery(1) = a.minTime(1)-2*charging_period-sum(traj_segment([1,2]));
% last_battery(2) = a.minTime(2)-1*charging_period-sum(traj_segment([3]));
% last_battery(3) = a.minTime(3)-2*charging_period-sum(traj_segment([4,5]));
% 
% [longest_time, wait_uav_idx] = max(last_battery);  % remove a waiting time for the longest operation uav
% last_battery(wait_uav_idx) = last_battery(wait_uav_idx) - charging_period;
% 
% 
% % Find the indices of negative elements
% % a.charger_travel_time % real travel time  usually small
% % a.time_given_charger % time limit         usually big
% interval = a.time_given_charger - a.charger_travel_time; % should be all positve. if not, the auv should wait
% negative_indx = find(interval < 0);
% 
% if length(negative_indx)>0   % 1 2 3 (asv1)   4 5 6(asv2)
%     disp('please insert waiting time')
% end
% 
% % 1 2 3 (asv1)   4 5 6(asv2)
% %auv_delay6 = a.charger_travel_time(6) - a.time_given_charger(6);  % 6 (asv2)
% auv_delay2 = a.charger_travel_time(2) - a.time_given_charger(2);  % 2 (asv1)
% 
% if auv_delay2 < 0
%     auv_delay2 = 0;
% end
%     
% if auv_delay6 < 0
%     auv_delay6 = 0;
% end

% auv2_delay = a.charger_travel_time(2) - a.time_given_charger(2);


% AUV1 = [traj_segment(1) charging_period                            traj_segment(2) charging_period last_battery(1)];
% AUV2 = [traj_segment(3) charging_period+auv2_delay charging_period traj_segment(4) charging_period last_battery(2)];
% AUV3 = [traj_segment(5) charging_period                            traj_segment(6) charging_period last_battery(3)];

% AUV3 = [traj_segment(5) charging_period                              traj_segment(6) charging_period last_battery(3)];
% AUV2 = [traj_segment(3) charging_period   charging_period            traj_segment(4) charging_period last_battery(2)];
% AUV1 = [traj_segment(1) charging_period                              traj_segment(2) charging_period last_battery(1)];


% 
% len_auv1 = length(AUV1) ;% 5
% len_auv2 = length(AUV2); % 5
% len_auv3 = length(AUV3); % 6


%% save time

%save('auv_time.mat', 'AUV1', 'AUV2', 'AUV3');   

%%


% figure
% 
% c = categorical({'AUV1','AUV2','AUV3'}); 
% 
% % schedule_tolerance = [0.0420    0.0767    0.2    0.2    0.2    0.0497]'; % auto
% % rest_delay = 0.2-schedule_tolerance;
% 
% % schedule_y = [traj_segment',schedule_tolerance,rest_delay];
% % schedule_y = zeros(9,1);
% % schedule_y([1,2,4,5,7,8]) = traj_segment(1:6)';
% % schedule_y([3,6,9]) = last_battery_life_time';
% 
% schedule_y = zeros(3,len_auv1+len_auv2+len_auv3);
% % schedule_y(1,1:5) = AUV1;
% % schedule_y(2,6:10) = AUV2;
% % schedule_y(3,11:end) = AUV3;
% 
% 
% schedule_y(1,1:len_auv1) = AUV1;
% schedule_y(2,len_auv1+1:len_auv1+len_auv2) = AUV2;
% schedule_y(3,len_auv1+len_auv2+1:end) = AUV3;



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
purpl = [1 0 1];


% colors_AUV3 = [purpl; black; green; purpl; black; orang; purpl];
% colors_AUV2 = [blue_; orang; blue_; green; blue_];
% colors_AUV1 = [red__; green; red__; orang; red__];


% colors_AUV3 = [purpl; black; green; purpl; black; orang; purpl];
% colors_AUV2 = [blue_; green; blue_; orang; blue_];
% colors_AUV1 = [red__; orang; red__; green; red__];

% colors_AUV3 = [purpl; orang; purpl; green; purpl];
% colors_AUV2 = [blue_; black; green; blue_; orang; blue_];
% colors_AUV1 = [red__; green; red__; orang; red__];
% 
% colors = [colors_AUV1; colors_AUV2; colors_AUV3];
% 
% % Plot stacked bar with specified colors
% schedule_bar = barh(c, schedule_y, 'stacked','FaceColor','flat');
% % for i = 1:size(schedule_y, 1) % 3
% %     for j = 1:size(schedule_y, 2) % 16
% %         schedule_bar(j).FaceColor = colors(i, :);
% %     end
% % end
% 
% 
% % bar (X)  CData(Y)  
% 
% for i = 1:size(colors_AUV1, 1)
%     schedule_bar(i).CData(1,:) = colors_AUV1(i, :);
% end
% 
% for i = 1:size(colors_AUV2, 1) %6~11
%     %schedule_bar(5+i).CData(2,:) = colors_AUV2(i, :);
%     schedule_bar(len_auv1+i).CData(2,:) = colors_AUV2(i, :);
% end
% 
% for i = 1:size(colors_AUV3, 1) %12
%     %schedule_bar(11+i).CData(3,:) = colors_AUV3(i, :);
%     schedule_bar(len_auv1+len_auv2+i).CData(3,:) = colors_AUV3(i, :);
% end
% 
% 
% 
% % Add legend with color labels
% % legend({'Segment 1','Segment 2','Segment 3','Segment 4','Segment 5','Segment 6'}, 'Location', 'SouthOutside', 'Orientation', 'horizontal');
% % Adjust color labels on the y-axis
% yticklabels({'AUV1','AUV2','AUV3'});
% 
% 
% % legend('boxoff')
% % legend('AUV#1','AUV#2','AUV#3','charger#1','charger#2','waiting')
% xlabel('Time (h)')


%%
figure

auv_colors = [red__;blue_;purpl];
nth_current_time = zeros(1,a.nSalesmen);


 % charger 
asv_colors = [green; orang];
charger_location_ind = a.charger_location_ind_record;
h1_flag = 0;
h2_flag = 0;
charging_period_start = a.charging_period_start;           % 9.6094   17.7475   27.6903    9.8855   27.3570   35.7475
time_given_charger = a.time_given_charger;                 % 9.6094    0.1381    1.9427    9.8855    9.4714    0.3906
charger_travel_time =  a.charger_travel_time;              % 0.9419    0.4161    0.6875    0.6035    0.4161    0.4268 
nth_worker_record = a.nth_worker_record;                   % 1           2        3         3          1        2
interval = a.time_given_charger - a.charger_travel_time;  % 8.6675   -0.2779    1.2552    9.2820    9.0554   -0.036



for ic = 1:numCharger  
    for jc = charger_location_ind(ic,1):charger_location_ind(ic,2)
        nth_charger(jc) = ic;                    %  1     1     1     2     2     2
    end
end

[scheduled_start, scheduled_ind] = sort(charging_period_start); % sort time schedule
scheduled_end = scheduled_start + charging_period; 
scheduled_nth_worker = nth_worker_record(scheduled_ind);
scheduled_nth_charger = nth_charger(scheduled_ind);



%% calculate waiting time 

% use nth_charger to decide USVs' first_charging index
unique_numbers = unique(nth_charger);
% len(unique_numbers) should be eqaul to the number of charger

working_robot_id = 1:1:a.nSalesmen;
remove_idx = [];
for i = 1:length(unique_numbers)
    number = unique_numbers(i);
    index = find(nth_charger == number, 1, 'first');
    % fprintf('The index of the first occurrence of %d is %d\n', number, index);
    charger_first_go_to_nth_worker = nth_worker_record(index); %  first visited working robots
    remove_idx = [remove_idx charger_first_go_to_nth_worker];
end
working_robot_id(remove_idx)=[]; % remove the first visisted working robot
wait_uav_idx = working_robot_id; % use the index to check which auv need to wait,
%% calcualte the last time 

% traj_worker_nth = 1     1     2     3     3

for i = 1:a.nSalesmen
    last_battery(i) = a.minTime(i);
end

for i = 1:length(traj_worker_nth)
    nth_worker_robot = traj_worker_nth(i);
    last_battery(nth_worker_robot) = last_battery(nth_worker_robot)-1*charging_period - traj_segment(i);
end
% last_battery(1) = a.minTime(1)-2*charging_period-sum(traj_segment([1,2]));
% last_battery(2) = a.minTime(2)-1*charging_period-sum(traj_segment([3]));
% last_battery(3) = a.minTime(3)-2*charging_period-sum(traj_segment([4,5]));

last_battery(wait_uav_idx) = last_battery(wait_uav_idx) - charging_period;  % the robot needs to wait so minus the last time
%%



for i = 1:a.nSalesmen   
    nth_auv = i;
    arrived_time = nth_current_time(i);
    
    index = find(traj_worker_nth == i, 1, 'first');
    nth_current_time(i) = nth_current_time(i) + traj_segment(index);
    % nth_current_time(i) = nth_current_time(i) + traj_segment(1+(i-1)*2);  % 1 3 5    % 1 4 5
    ended_time = nth_current_time(i);
    xy_1 = [arrived_time nth_auv];
    xy_2 = [ended_time nth_auv];
    line = [xy_1;xy_2];  
    plot(line(:,1),line(:,2),'linewidth',10,'Color',auv_colors(i,:)); 
    hold on;
end



% add waiting time for the   which auv need to wait,
    nth_auv = wait_uav_idx;
    i = nth_auv; 
    arrived_time = nth_current_time(i);
    nth_current_time(i) = nth_current_time(i) + charging_period;
    ended_time = nth_current_time(i);
    xy_1 = [arrived_time nth_auv];
    xy_2 = [ended_time nth_auv];
    line = [xy_1;xy_2];  
    plot(line(:,1),line(:,2),'linewidth',5,'Color',black); 
    hold on;


scheduled_time_given_charge = time_given_charger(scheduled_ind);
scheduled_charger_travel_time = charger_travel_time(scheduled_ind);
scheduled_interval = interval(scheduled_ind)
% traj_segment
% workers_time = [traj_segment(1:2) last_battery(1);
%                 traj_segment(3:4) last_battery(2);
%                 traj_segment(5:6) last_battery(3)];  % 3 workers  x 3 time slot 

workers_time = { [traj_segment(1:2), last_battery(1)];
                 [traj_segment(3), last_battery(2)];
                 [traj_segment(4:5), last_battery(3)]};  % 3 workers  x 3 time slot 
workers_indx = [2 2 2];

for i = 1:length(scheduled_ind)
    
    
    nth_auv = scheduled_nth_worker(i); % y   % 1 ~ 3 
    nth_charger_color = scheduled_nth_charger(i); % color

    if scheduled_interval(i) < 0
        % insert waitting time
        add_waiting_time = -1*scheduled_interval(i);
        
        arrived_time = nth_current_time(nth_auv);
        nth_current_time(nth_auv) = nth_current_time(nth_auv) + add_waiting_time; % push the nth auv to wait 
        ended_time = nth_current_time(nth_auv);
        xy_1 = [arrived_time nth_auv];
        xy_2 = [ended_time nth_auv];
        line = [xy_1;xy_2]; 
        plot(line(:,1),line(:,2),'linewidth',10,'Color',black,'HandleVisibility','off');  % 
        hold on;
        
        % update schedule_start
        % find the index auv2 and push the time (add_waiting_time)
        auv_delay_indices = find(scheduled_nth_worker == nth_auv);
        for k = 1:length(auv_delay_indices)
            idx = auv_delay_indices(k);
            scheduled_start(idx) = scheduled_start(idx) + add_waiting_time;
            scheduled_end(idx) = scheduled_end(idx) + add_waiting_time;
        end
        
        
        % how to update the interval again after i 
        
        for w = i+1 :length(interval)  % 4~6  only the consider the rest time           
            value_to_find = scheduled_nth_charger(w);  %  1     2     1     2     1     2
            % Find the indices of the same value before the w th element
            indices = find(scheduled_nth_charger(1:w-1) == value_to_find);
            last_moment_charger_indice = indices(end);    % previous end time       
            scheduled_time_given_charge(w) = scheduled_start(w) - scheduled_end(last_moment_charger_indice);
        end
        disp("update interval")
        scheduled_interval = scheduled_time_given_charge - scheduled_charger_travel_time
        
        % update start time
        % update end time
        
        % time_given_charger = start_time - end_time
        % interval = time_given_charger - charger_travel_time;  % - % constant
        
    end
    
    
    arrived_time = scheduled_start(i); % x
    ended_time = arrived_time  + charging_period;
    xy_1 = [arrived_time nth_auv];
    xy_2 = [ended_time nth_auv]; 
    line = [xy_1;xy_2];
    %plot(line(:,1),line(:,2),'linewidth',10,'Color',asv_colors(nth_charger_color,:));
    if h1_flag == 0 && nth_charger_color == 1
        plot(line(:,1),line(:,2),'linewidth',10,'Color',asv_colors(nth_charger_color,:)); % charger # 1
        h1_flag = 1;
    elseif h2_flag == 0 && nth_charger_color == 2
        plot(line(:,1),line(:,2),'linewidth',10,'Color',asv_colors(nth_charger_color,:)); % charger # 1
        h2_flag = 1;            
    else
        plot(line(:,1),line(:,2),'linewidth',10,'Color',asv_colors(nth_charger_color,:),'HandleVisibility','off');
    end    

 
    idx = workers_indx(nth_auv);  % read index
    %working_len = workers_time(nth_auv,idx);  % read working length
    working_len = workers_time{nth_auv}(idx);  % read working length
    arrived_time = ended_time;
    ended_time = arrived_time + working_len;
    xy_1 = [arrived_time nth_auv];
    xy_2 = [ended_time nth_auv]; 
    line = [xy_1;xy_2];    
    plot(line(:,1),line(:,2),'linewidth',10,'Color',auv_colors(nth_auv,:),'HandleVisibility','off');
    
    workers_indx(nth_auv) = workers_indx(nth_auv)+1; % update index 
    nth_current_time(nth_auv) = ended_time; % update_time
end



% for ic = 1:numCharger   
%     for jc = charger_location_ind(ic,1):charger_location_ind(ic,2)        
%         nth_auv = a.nth_worker_record(jc); %y        
%         if interval(jc) < 0  % nth_auv need to wait, and push the later schedule and update the interval
%             arrived_time = nth_current_time(nth_auv);
%             add_waiting_time = -1*interval(jc);
%             nth_current_time(nth_auv) = nth_current_time(nth_auv) + add_waiting_time; % push the nth auv to wait 
%             ended_time = nth_current_time(nth_auv);
%             % please update all thread
%             xy_1 = [arrived_time nth_auv];
%             xy_2 = [ended_time nth_auv];
%             line = [xy_1;xy_2];             
%             plot(line(:,1),line(:,2),'linewidth',10,'Color',"cyan",'HandleVisibility','off'); 
%         end              
%         arrived_time = charging_period_start(jc); % x
%         ended_time = arrived_time  + charging_period;
%         %nth_current_time(nth_auv) = ended_time;
%         
%         xy_1 = [arrived_time nth_auv];
%         xy_2 = [ended_time nth_auv];
%         line = [xy_1;xy_2];
%         
%         if h1_flag == 0 && ic == 1
%             plot(line(:,1),line(:,2),'linewidth',10,'Color',asv_colors(ic,:)); % charger # 1
%             h1_flag = 1;
%         elseif h2_flag == 0 && ic == 2
%             plot(line(:,1),line(:,2),'linewidth',10,'Color',asv_colors(ic,:)); % charger # 1
%             h2_flag = 1;            
%         else
%             plot(line(:,1),line(:,2),'linewidth',10,'Color',asv_colors(ic,:),'HandleVisibility','off'); 
%         end
%         hold on;        
%     end
% end    
legend('AUV#1','AUV#2','AUV#3','waiting','charger#1','charger#2')
% , 'HandleVisibility','off'


ylim([0.9, 3.1]);
yticks(1:3);
yticklabels({'AUV1','AUV2','AUV3'});
xlabel('Time (h)')


title("AUV working and charging period")
%ax.Position = [0.03, 0.041, 0.95, 0.95]; % [left, bottom, width, height]

%% Save ASV travel time
asv1_index = 0;
asv2_index = 0;
asv1_travel = [];
asv2_travel = [];
for i = 1:length(scheduled_ind)
    
    nth_charger_color = scheduled_nth_charger(i); % y   1 or 2
    
    if nth_charger_color == 1 
        if asv1_index == 0
            start_time = 0;
        else
            value_to_find = scheduled_nth_charger(i);  %  1     2     1     2     1     2
            % Find the indices of the same value before the i th element
            indices = find(scheduled_nth_charger(1:i-1) == value_to_find);
            last_moment_charger_indice = indices(end);    % previous end time    
            start_time = scheduled_end(last_moment_charger_indice);
        end
        asv1_index = asv1_index+1; 
        end_time = scheduled_charger_travel_time(i);
        travel_range = [start_time start_time+end_time];
        asv1_travel = [asv1_travel; travel_range];
    elseif nth_charger_color == 2 
        if asv2_index == 0
            start_time = 0; 
        else
            value_to_find = scheduled_nth_charger(i);  %  1     2     1     2     1     2
            % Find the indices of the same value before the i th element
            indices = find(scheduled_nth_charger(1:i-1) == value_to_find);
            last_moment_charger_indice = indices(end);    % previous end time    
            start_time = scheduled_end(last_moment_charger_indice);            
        end
        asv2_index = asv2_index+1;
        end_time = scheduled_charger_travel_time(i);
        travel_range = [start_time start_time+end_time];
        asv2_travel = [asv2_travel; travel_range];
    end      
end


asv_travel(1).data = asv1_travel;
asv_travel(2).data = asv2_travel;


%% save file

fileName = ['asv_travel.mat'];
save(fileName, 'asv_travel');



% a.minTime does not consider that the UAV can not arrive in time 
