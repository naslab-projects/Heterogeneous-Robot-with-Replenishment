% Define the coordinates
x1 = [1 9];
y1 = [2 7];
k = [x1; y1];
% x2 = [1 2];
% y2 = [1 1];
% 
% x3 = [1 2];
% y3 = [1 2];

% Plot the lines
figure;
hold on;

plot(k(:,1),k(:,2), '-o', 'LineWidth', 2, 'MarkerSize', 8);
% plot(x2, y2, '-s', 'LineWidth', 2, 'MarkerSize', 8);
% plot(x3, y3, '-^', 'LineWidth', 2, 'MarkerSize', 8);

% Set axis labels and title
xlabel('X-axis');
ylabel('Y-axis');
title('Multiple Lines Connecting to (1,1)');

% Add a legend
legend('To (1,2)', 'To (2,1)', 'To (2,2)');

% Display the grid
grid on;

% Hold off to stop adding to the current plot
hold off;