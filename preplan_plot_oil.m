function [traj_segment,time_given_charger] = preplan_plot_oil(a,numTarChargers,userConfig,start_point,start_pCharger)
%% Plot
N = length(a.optRoute);
rng = [[1 a.optBreak+1];[a.optBreak N]]';
clr = hsv(a.nSalesmen+5);
figure
hAx = gca;
hold on
scala = 5;

batteryLife = a.charging_level;
% battery_history = zeros(a.nSalesmen,560);
man_battery_count = zeros(1,a.nSalesmen);
count = 1;
battery_life_count = 1;
% evaluate the mission
for s = 1:a.nSalesmen
    d = userConfig.start_mat(a.optRoute(rng(s,1)),1); % add starting point
    time = d/userConfig.start_mat(a.optRoute(rng(s,1)),2);
    time_charging = time;
%     battery_history(s,ceil(time)) = 20 - time;
    for k = rng(s,1):rng(s,2)-1
        time_charging = time_charging + a.dmat(a.optRoute(k),a.optRoute(k+1))/userConfig.delta_v;
        if time_charging > batteryLife(s,man_battery_count(s)+1) % if the battery is enough to make this travel
            battery_life_time(battery_life_count) = time_charging - a.dmat(a.optRoute(k),a.optRoute(k+1))/userConfig.delta_v; % evaluate each trajectory segment
            time_start_charging(battery_life_count) = time; % time to start charging
%             battery_history(s,ceil(time_start_charging)) = 20 - time_charging;
            time = time + userConfig.charging_time; % add charging period TODO: add a ratio for charging ratio
            time_end_charging(battery_life_count) = time; % time finished charging
%             battery_history(s,ceil(time_end_charging(battery_life_count))) = 20;
            time_charging = a.dmat(a.optRoute(k),a.optRoute(k+1))/userConfig.delta_v;
            index_Stations(battery_life_count) = count;
            battery_life_count = battery_life_count + 1;
            man_battery_count(s) = man_battery_count(s) + 1;
        end
        d = d + a.dmat(a.optRoute(k),a.optRoute(k+1)); % distance
        time = time + a.dmat(a.optRoute(k),a.optRoute(k+1))/userConfig.delta_v;
%         battery_history(s,ceil(time)) = 20 - time_charging ; % battery life at time step
        count = count + 1;
    end
    count = count + 1;
    
    temp(s)=d;
    time_temp(s)=time;
    totalTime = max(time_temp);
%     battery_history = battery_history(:,1:ceil(totalTime));
    rte = a.optRoute(rng(s,1):rng(s,2));
    plot(hAx,[start_point(1,1); a.xy(rte,1)]*scala,[start_point(1,2); a.xy(rte,2)]*scala,'-','linewidth',2,'Color',clr(s,:));
    %         plot(hAx,[a.xy(rte,1)],[a.xy(rte,2)],'.-','linewidth',2,'Color',clr(s,:));
    hold(hAx,'on');
end
time_temp
% fprintf('total energy =')
% disp(sum(time_temp))

%% Plot chargers

% charger marker
% C = char('s','o');

[scheduled_start, scheduled_ind] = sort(time_start_charging); % sort time schedule
pStation = a.optRoute(index_Stations);
scheduled_charger = pStation(scheduled_ind);
% scheduled_start = time_start_charging(scheduled_ind);
scheduled_end = time_end_charging(scheduled_ind);

num_chargering_period = length(a.charging_location);
NUm = ones(1,numTarChargers)*floor(num_chargering_period/numTarChargers);
remainder_NUm = rem(num_chargering_period,numTarChargers);
NUm(1:remainder_NUm) = NUm(1:remainder_NUm) + 1;

ic_count = 1;
new_order_pStation = zeros(1,num_chargering_period);
time_start = zeros(1,num_chargering_period);
time_end = zeros(1,num_chargering_period);
% scheduling for chargers
for ic = 1:numTarChargers
    for jc = 1:NUm(ic)
        new_order_pStation(ic_count) = scheduled_charger((jc-1)*numTarChargers+ic);
        time_start(ic_count) = scheduled_start((jc-1)*numTarChargers+ic);
        time_end(ic_count) = scheduled_end((jc-1)*numTarChargers+ic);
        ic_count = ic_count + 1;
    end
end

%% Calculate charger energy
time_charger = zeros(numTarChargers,40);
time_charger_travel = zeros(1,length(new_order_pStation)-numTarChargers+1);
time_end_start = zeros(1,length(new_order_pStation)-numTarChargers+1);
cum_NUm = cumsum(NUm);
charger_location_ind = [[1,cum_NUm(1:end-1)+1];cum_NUm]';
charger_count = 1;

for ic = 1:numTarChargers
    charger_energy_count = 1;
    time_charger_travel(charger_count) = userConfig.start_mat(new_order_pStation(charger_location_ind(ic,1)),2*1+ic)/userConfig.delta_vc;
    time_end_start(charger_count) = time_start(charger_location_ind(ic,1));
    charger_count = charger_count + 1;
    charger_energy_count = charger_energy_count + 1;
    for jc = charger_location_ind(ic,1):charger_location_ind(ic,2)-1
        time_charger_travel(charger_count) = a.dmat(new_order_pStation(jc),new_order_pStation(jc+1))/userConfig.delta_vc;
        time_charger(ic,charger_energy_count) = time_charger_travel(charger_count);
        time_end_start(charger_count) = time_start(jc+1) - time_end(jc);
        charger_count = charger_count + 1;
        charger_energy_count = charger_energy_count + 1;
    end
end

time_given_charger = time_end_start;
% cum_NUm = cumsum(NUm);
% charger_location_ind = [[1,cum_NUm(1:end-1)+1];cum_NUm]';
count_p = 1;

% for ic = 1:numTarChargers
%     plot(start_pCharger(ic,1),start_pCharger(ic,2),C(ic),'linewidth',2,'MarkerSize',20,'MarkerEdgeColor',clr(s+ic,:))
%     text(start_pCharger(ic,1)-.2,start_pCharger(ic,2),'1','color',clr(s+ic,:),'FontSize',14)
% end
% for ic = 1:numTarChargers
%     for id = 1:1
%         plot(a.xy(a.charging_location(count_p),1), a.xy(a.charging_location(count_p),2),C(ic),'linewidth',2,'MarkerSize',20,'MarkerEdgeColor',clr(s+ic,:))
% %         text(a.xy(a.charging_location(count_p),1)-.2,a.xy(a.charging_location(count_p),2),num2str(id),'color',clr(s+ic,:),'FontSize',14)
%         count_p = count_p +1;
%     end
% end
% count_p = 1;
% plot([0.5,14.5,14.5,0.5,.5],[0.5,.5,14.5,14.5,.5],'k','LineWidth',3)


plot([1.5,1.5,10.5,10.5,15.5,15.5,16.5,16.5,18.5,18.5,14.5,14.5,12.5,12.5,9.5,9.5,7.5,7.5,4.5,4.5,2.5,2.5,1.5]*scala,[3.5,10.5,10.5,9.5,9.5,8.5,8.5,7.5,7.5,1.5,1.5,2.5,2.5,3.5,3.5,1.5,1.5,2.5,2.5,1.5,1.5,3.5,3.5]*scala,'k','linewidth',2)
xlabel('X (km)')
ylabel('Y (km)')
% o_x = [2,2,2,2,3,3,3,4,4,4,5,5,6,7,7,8,8,8,9,9,12,13];
% o_y = [5:8,5:7,5:7,5,6,6,6,7,6,7,8,6,7,5,5];
% o_x = [2 6 9 10 11 12 14 5 6 7];
% o_y = [6 6 6 6 6 6 6 5 5 5];
o_x = [5 6 7 10:14 10 11 12 14 17 18];
o_y = [2 2 2 2 2 2 2 2 3 3 3 3 2 2];
% scatter(o_x*scala,o_y*scala,'yd','filled')
% for ic = 1:numTarChargers
%     for id = 1:NUm(ic)
%         plot(a.xy(a.charging_location(count_p),1), a.xy(a.charging_location(count_p),2),C(ic),'linewidth',2,'MarkerSize',20,'MarkerEdgeColor',clr(s+ic,:))
% %         text(a.xy(a.charging_location(count_p),1)-.2,a.xy(a.charging_location(count_p),2),num2str(id),'color',clr(s+ic,:),'FontSize',14)
%         count_p = count_p +1;
%     end
% end

for ic = 1:numTarChargers
    
       plot([start_pCharger(1,1); a.xy(a.charging_location(charger_location_ind(ic,1):charger_location_ind(ic,2)),1)]*scala, ... 
           [start_pCharger(1,2); a.xy(a.charging_location(charger_location_ind(ic,1):charger_location_ind(ic,2)),2)]*scala,'--x','LineWidth',2,'MarkerSize',15,'Color',clr(s+ic,:)) 
    
end

plot(start_point(1,1)*scala,start_point(1,2)*scala,'k*','MarkerSize',15,'LineWidth',3)

% plot(a.xy(a.charging_location,1),a.xy(a.charging_location,2),'kx','LineWidth',2,'MarkerSize',20)
axis equal
% legend('AUV#1','AUV#2','AUV#3','border','charger#1','charger#2','start')
% legend('boxoff')
% plot(a.xy(a.charging_location,1),a.xy(a.charging_location,2),'s','MarkerSize',10,'MarkerEdgeColor',clr(s+ic,:))
xlabel('X (km)')
ylabel('Y (km)')

traj_segment = battery_life_time;
end