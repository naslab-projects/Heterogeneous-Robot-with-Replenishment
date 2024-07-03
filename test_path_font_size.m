% Create a graph
s = [1 2 2 3 4];
t = [2 3 4 1 3];
weights = [50 2 7 1 4];

G = graph(s, t, weights);

% Plot the graph with EdgeLabel and adjust font size
h = plot(G, 'EdgeLabel', G.Edges.Weight);

% Change the size of edge labels
%h.EdgeLabel.FontSize = 12; % Set the desired font size
%Unable to perform assignment because dot indexing is not supported for variables of this type.


% Change the font size of the edge labels
%set(h.EdgeLabel,'FontSize',14)

% Change the font size of the edge labels
%set(h.EdgeLabel,'FontSize',14);

h.EdgeFontSize = 15;
set(groot, 'DefaultAxesFontSize', 5); % Set font size to 14 for axis labels and ticks
set(groot, 'DefaultTextFontSize', 5); % Set font size to 16 for title and text annotations

% Manually adjust margins
ax = gca;
ax.Position = [0.05, 0.09, 0.95, 0.95]; % [left, bottom, width, height]

xlabel('X (km)')
ylabel('Y (km)')
