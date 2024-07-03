% Example plot
x = linspace(0, 2*pi, 100);
y = sin(x);

figure;
h = plot(x, y);
title('Example Plot');

% Manually adjust margins
ax = gca;
ax.Position = [0.08, 0.08, 0.9, 0.9]; % [left, bottom, width, height]