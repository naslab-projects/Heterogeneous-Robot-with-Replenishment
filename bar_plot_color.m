% y = [50 50 30; 25 75 20; 30 70 20];   % 3 x 2
% figure
% c = categorical({'AUV1','AUV2','AUV3'}); 
% ba = barh(c, y,'stacked', 'FaceColor','flat');
% ba(1).CData = [1 0 0.7];
% ba(2).CData = [1 1 1]*0.8;

clc;
clear all;
close all;



AUV1 = [9.70770187520589,8,9.74754689570643,8,4.21676051329096];
AUV2 = [9.88342717995763,8.27928156332829,8,9.80473785412436,8,2.66666666666665];
AUV3 = [9.74535599249993,8,10,8,2.66666666666665];

figure
c = categorical({'AUV1','AUV2','AUV3'}); 
schedule_y = zeros(3,16);
% schedule_y(1,1:5) = AUV1;
% schedule_y(2,1:6) = AUV2;
% schedule_y(3,1:5) = AUV3;
schedule_y(1,1:5) = AUV1;
schedule_y(2,6:11) = AUV2;
schedule_y(3,12:end) = AUV3;

schedule_bar = barh(c,schedule_y,'stacked','FaceColor','flat');


clr = [1 0 0;1 0 0; 1 0 0; 0 0 1;0 0 1; 0 0 1;1 0 1;1 0 1;1 0 1];


% schedule_bar(1).FaceColor = 'flat';
schedule_bar(1).CData(1,:) = [1 0 0];    % bar (X)  CData(Y)    
schedule_bar(2).CData(1,:) = [1 0 0];
schedule_bar(3).CData(1,:) = [1 0 0];
schedule_bar(4).CData(1,:) = [1 0 0];
schedule_bar(5).CData(1,:) = [1 0 0];
% schedule_bar(6).FaceColor = 'flat';
schedule_bar(10).CData(2,:) = [1 0 0];
schedule_bar(9).CData(2,:) = [1 0 0];

% % schedule_bar(2).FaceColor = 'y';
% % schedule_bar(3).FaceColor = 'g';
% for i = 1:9
%     schedule_bar(1).CData(i,:) = clr(i,:);
% end


