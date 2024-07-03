
AUV1 = [9 charging_period charging_period 9 charging_period];
AUV2 = [10 charging_period charging_period 9 2.5 charging_period];
AUV3 = [10 charging_period 10 charging_period 9 charging_period 9 charging_period 8.7 charging_period 4.7];
AUV4 = [9.4 charging_period 9 charging_period 9.7 charging_period 8.3 charging_period 9 charging_period 3.3];
AUV5 = [9];
AUV6 = [8.7 charging_period 10 charging_period 10 charging_period 9.7 charging_period 10 charging_period 3.7];
AUV7 = [8.5 charging_period 9 5 charging_period 10 charging_period 10 charging_period 6.7];

figure

c = categorical({'AUV1','AUV2','AUV3','AUV4','AUV5','AUV6','AUV7'}); 

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
legend('AUV#1','AUV#2','AUV#3','charger#1','charger#2','waiting')
legend('boxoff')
xlabel('Time (h)')