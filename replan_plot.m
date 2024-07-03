function replan_plot(a,numTarChargers,userConfig,start_point,start_pCharger)

%% Plot 
N = length(a.optRoute);
rng = [[1 a.optBreak+1];[a.optBreak N]]';
clr = [1 0 0; 0 0 1; 1 0 1;  1 0.5 0;0 1 0; .5 .5 0 ;0 0 0; 1 .5 .5];
clr = [clr;zeros(10,3)];
figure
hAx = gca;
hold on

% batteryLife = ones(a.nSalesmen,50)*userConfig.batteryLife;
% batteryLife(:,1) = (1:a.nSalesmen)*floor(userConfig.batteryLife/a.nSalesmen);
batteryLife = a.charging_level;
battery_history = zeros(a.nSalesmen,560);
man_battery_count = zeros(1,a.nSalesmen);
count = 1;
battery_life_count = 1;
% evaluate the mission
for s = 1:a.nSalesmen
		d = userConfig.start_mat(a.optRoute(rng(s,1)),1); % add starting point
%         time = d/userConfig.start_mat(a.optRoute(rng(s,1)),2); % add time start point
%         d = a.dmat(a.optRoute(rng(s,1)),83)+.6;
%         d = .01;
        time = d/userConfig.start_mat(a.optRoute(rng(s,1)),2);
        time_charging = time;
        battery_history(s,ceil(time)) = 20 - time;
         %add the starting point
%         plot(start_point(1,1),start_point(1,2),'go')
        for k = rng(s,1):rng(s,2)-1
            time_charging = time_charging + a.dmat(a.optRoute(k),a.optRoute(k+1))/userConfig.delta_v;
            
            if time_charging > batteryLife(s,man_battery_count(s)+1) % if the battery is enough to make this travel
                battery_life_time(battery_life_count) = time_charging - a.dmat(a.optRoute(k),a.optRoute(k+1))/userConfig.delta_v;
                time_start_charging(battery_life_count) = time; % time to start charging
%                 battery_history(s,ceil(time_start_charging)) = 20 - time_charging;
                time = time + userConfig.charging_time; % add charging period TODO: add a ratio for charging ratio
                time_end_charging(battery_life_count) = time; % time finished charging
                battery_history(s,ceil(time_end_charging(battery_life_count))) = 20;
                time_charging = a.dmat(a.optRoute(k),a.optRoute(k+1))/userConfig.delta_v;
                index_Stations(battery_life_count) = count;
                battery_life_count = battery_life_count + 1;
                man_battery_count(s) = man_battery_count(s) + 1;
                
            end
            d = d + a.dmat(a.optRoute(k),a.optRoute(k+1)); % distance
            time = time + a.dmat(a.optRoute(k),a.optRoute(k+1))/userConfig.delta_v;
            battery_history(s,ceil(time)) = 20 - time_charging ; % battery life at time step
            count = count + 1;
            
        end
        count = count + 1;
        
        temp(s)=d;
        time_temp(s)=time;
        totalDist = sum(temp);
        totalTime = max(time_temp);
        battery_history = battery_history(:,1:ceil(totalTime));
        rte = a.optRoute(rng(s,1):rng(s,2));
        plot(hAx,[start_point(s,1); a.xy(rte,1)],[start_point(s,2); a.xy(rte,2)],'--','linewidth',2,'Color',clr(s,:));
%         plot(hAx,[a.xy(rte,1)],[a.xy(rte,2)],'.-','linewidth',2,'Color',clr(s,:));
        hold(hAx,'on');
end
fprintf('total energy =')
disp(time_temp)


%% Plot chargers

C = char('o','s');

num_chargering_period = length(a.charging_location);
NUm = ones(1,numTarChargers)*floor(num_chargering_period/numTarChargers);
remainder_NUm = rem(num_chargering_period,numTarChargers);
NUm(1:remainder_NUm) = NUm(1:remainder_NUm) + 1;
% cum_NUm = cumsum(NUm);
% charger_location_ind = [[1,cum_NUm(1:end-1)+1];cum_NUm]';
count_p = 1;

C = char('s','o');
clr = [1,0,0;0,0,1;1,0,1;0,1,0;1,0.500000000000000,0];
s = 3;
for ic = numTarChargers:-1:1
     plot(start_pCharger(ic,1),start_pCharger(ic,2),C(ic),'linewidth',2,'MarkerSize',20,'MarkerEdgeColor',clr(s+ic,:),'MarkerFaceColor',clr(s+ic,:))
     text(start_pCharger(ic,1)-.2,start_pCharger(ic,2),'2','color','w','FontSize',14)
end
plot([0.5,14.5,14.5,0.5,.5],[0.5,.5,14.5,14.5,.5],'k','LineWidth',3)

C = char('o','s');
clr = [1,0,0;0,0,1;1,0,1;1,0.500000000000000,0;0,1,0];
% NUm(1)= 1;
for ic = 1:numTarChargers
    for id = 1:NUm(ic)
       plot(a.xy(a.charging_location(count_p),1), a.xy(a.charging_location(count_p),2),C(ic),'linewidth',2,'MarkerSize',20,'MarkerEdgeColor',clr(s+ic,:))
       text(a.xy(a.charging_location(count_p),1)-.2,a.xy(a.charging_location(count_p),2),num2str(id+2),'color',clr(s+ic,:),'FontSize',14)
       count_p = count_p +1;
    end
end


axis([0,18,0,15])


end