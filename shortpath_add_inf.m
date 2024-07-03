clc;
clear all;
close all;


%% new version 

% Create a 14x14 matrix
matrix = zeros(14, 14);
% Populate the matrix with consecutive numbers in each row
for i = 1:14
    matrix(i, :) = (i-1)*14 + 1 : i*14;
end
% Display the matrix
% disp('Matrix:');
% disp(matrix);

% Initialize arrays for s, t, and weight
s = [];
t = [];
weight = [];

% Iterate through the matrix to define connections
for i = 1:size(matrix, 1)
    for j = 1:size(matrix, 2)
        % Current node
        current_node = matrix(i, j);
        
        % Check right neighbor
        if j < size(matrix, 2)
            right_neighbor = matrix(i, j+1);
            s = [s current_node];
            t = [t right_neighbor];
            weight = [weight 1];
        end
        
        % Check lower neighbor
        if i < size(matrix, 1)
            lower_neighbor = matrix(i+1, j);
            s = [s current_node];
            t = [t lower_neighbor];
            weight = [weight 1];
        end
        
        % Check lower-right neighbor (diagonal)
        if i < size(matrix, 1) && j < size(matrix, 2)
            lower_right_neighbor = matrix(i+1, j+1);
            s = [s current_node];
            t = [t lower_right_neighbor];
            weight = [weight sqrt(2)]; % Diagonal distance
        end
        
        % Check lower-left neighbor (diagonal)
        if i < size(matrix, 1) && j > 1
            lower_left_neighbor = matrix(i+1, j-1);
            s = [s current_node];
            t = [t lower_left_neighbor];
            weight = [weight sqrt(2)]; % Diagonal distance
        end
    end
end



%% old version 

% % Create a 14x14 matrix
% matrix = zeros(14, 14);
% 
% % Populate the matrix with consecutive numbers in each row
% for i = 1:14
%     matrix(i, :) = (i-1)*14 + 1 : i*14;
% end
% 
% % Display the matrix
% disp('Matrix:');
% disp(matrix);
% 
% % Initialize arrays for s, t, and weight
% s = [];
% t = [];
% weight = [];
% 
% % Iterate through the matrix to define connections
% for i = 1:size(matrix, 1)
%     for j = 1:size(matrix, 2)
%         % Current node
%         current_node = matrix(i, j);
%         
%         % Check adjacent nodes
%         if j < size(matrix, 2)
%             % Right neighbor
%             right_neighbor = matrix(i, j+1);
%             s = [s current_node];
%             t = [t right_neighbor];
%             weight = [weight 1];
%         end
%         
%         if i < size(matrix, 1)
%             % Lower neighbor
%             lower_neighbor = matrix(i+1, j);
%             s = [s current_node];
%             t = [t lower_neighbor];
%             weight = [weight 1];
%         end
%         
%         if i < size(matrix, 1) && j < size(matrix, 2)
%             % Lower-right neighbor (diagonal)
%             lower_right_neighbor = matrix(i+1, j+1);
%             s = [s current_node];
%             t = [t lower_right_neighbor];
%             weight = [weight sqrt(2)]; % Diagonal distance
%         end
%     end
% end

% Display the result
% disp('s:');
% disp(s);
% disp('t:');
% disp(t);
% disp('weight:');
% disp(weight);




%% xy position
% Initialize arrays for x and y
x = [];
y = [];

% Number of rows and columns in the matrix
num_rows = 14;
num_cols = 14;

% Populate x and y arrays
for i = 1:num_rows
    for j = 1:num_cols
        x = [x j];
        y = [y i];
    end
end
% Display the result
% disp('x:');
% disp(x);
% disp('y:');
% disp(y);

%% add start point new node
x = [x 8];
y = [y 0];
s = [s 197 197 197];
t = [t 7 8 9];
weight = [weight 1.414 1 1.414];





%% remove a single one
% Example usage

% nodeToRemove = 35;
% [s, t, weight] = removeNode(s, t, weight, nodeToRemove);
% n = nodeToRemove;
% x = removeNthElement(x, n);
% y = removeNthElement(y, n);


%% this is fail because change the order
% points_to_remove = [6 4; 6 5; 6 6; 6 7; 7 4; 7 5; 7 6; 8 5; 8 6; 9 5; 10 5]; % island
% remove_node_arr = (points_to_remove(:,2)-1)*14+points_to_remove(:,1);% (y-1)*14+x = nth 
% remove_node_arr = sort(remove_node_arr) % small to big
% % Automatically generate vector b based on the length of a
% b = 0:(length(remove_node_arr)-1);
% % Perform element-wise subtraction
% remove_node_arr_minus_order = remove_node_arr - b';
% 
% for i = 1:length(remove_node_arr)
%     nodeToRemove = remove_node_arr_minus_order(i);
%     [s, t, weight] = removeNode(s, t, weight, nodeToRemove);
%     n = nodeToRemove;
%     x = removeNthElement(x, nodeToRemove);
%     y = removeNthElement(y, nodeToRemove); 
% end


%% add obstacle by increasing the weight
% Example usage
% nodesToSelected = 17;
increaseFactor = 10;

% Call the function to increase the weights of the specified node connections
% [s, t, weight] = increase_weights(s, t, weight, nodesToSelected, increaseFactor);

points_to_remove = [6 4; 6 5; 6 6; 6 7; 7 4; 7 5; 7 6; 8 5; 8 6;]; % island
remove_node_arr = (points_to_remove(:,2)-1)*14+points_to_remove(:,1);% (y-1)*14+x = nth 

for i = 1:length(remove_node_arr)
    nodesToSelected = remove_node_arr(i);
    [s, t, weight] = increase_weights(s, t, weight, nodesToSelected, increaseFactor);
end

scatter(points_to_remove(:,1),points_to_remove(:,2),'MarkerFaceColor',[0 .7 .7])
hold on

%% 
% points_to_remove = [6 4; 6 5]; % island
% remove_node_arr = (points_to_remove(:,2)-1)*14+points_to_remove(:,1);% (y-1)*14+x = nth 
% nodeToRemove = remove_node_arr;
% [s, t, weight] = removeNodes(s, t, weight, nodeToRemove);  
% x = removeNthElements(x, nodeToRemove);
% y = removeNthElements(y, nodeToRemove); 


% Call the function to remove the specified node


% Display the updated result
% disp('Updated s:');
% disp(s);
% disp('Updated t:');
% disp(t);
% disp('Updated weight:');
% disp(weight);




%%  destination update


% [path1,d] = shortestpath(G,3,15)
% highlight(p,path1,'EdgeColor','g')


start_p = [8 1];
end_p = [8 1];
start_p_indx = start_p(1)+(start_p(2)-1)*14;
end_p_indx = end_p(1)+(end_p(2)-1)*14;

G = graph(s,t,weight);
p = plot(G,'XData',x,'YData',y,'EdgeLabel',G.Edges.Weight);
[path1,d] = shortestpath(G,start_p_indx,end_p_indx)
highlight(p,path1,'EdgeColor','g','LineWidth',3)

scatter(8,0,'MarkerFaceColor','r')
hold on

%%
% number_front = countNumbersInFront(remove_node_arr, start_p_indx);  % modify the index because some node get removed by the obstacle
% start_p_indx_minus_empty = start_p_indx - number_front;
% 
% 
% number_front = countNumbersInFront(remove_node_arr, end_p_indx);
% end_p_indx_minus_empty = end_p_indx - number_front;
% 
% [path1,d] = shortestpath(G,start_p_indx_minus_empty,end_p_indx_minus_empty)
% highlight(p,path1,'EdgeColor',[0.8500 0.3250 0.0980] ,'LineWidth',3)
% grid on


%%

% Create the initial graph
G = graph(s,t,weight);

% Initialize the plot outside the loop
p = plot(G,'XData',x,'YData',y,'EdgeLabel',G.Edges.Weight);

% Loop over nodes
for i = 1:196
    % Find the shortest path
    [path1, d] = shortestpath(G, i, 2);
    
    % Clear the previous highlight color
    %highlight(p, 'off');
    
    % Highlight the path
    highlight(p, path1, 'EdgeColor', 'g', 'LineWidth', 3);
    
    title([ sprintf('%d', i) ' nth node of Distance: '  sprintf('%.2f', d)  ' meter']);
    
    
    % Pause to observe the plot (optional)
    pause(0.2);
end


%% single remove
function [s, t, weight] = removeNode(s, t, weight, nodeToRemove)
    % Find indices of connections involving the specified node
    indicesToRemove = (s == nodeToRemove) | (t == nodeToRemove);
    
    % Remove connections associated with the specified node
    s(indicesToRemove) = [];
    t(indicesToRemove) = [];
    weight(indicesToRemove) = [];
    
    % Update node numbers greater than the removed node
    s(s > nodeToRemove) = s(s > nodeToRemove) - 1;
    t(t > nodeToRemove) = t(t > nodeToRemove) - 1;
end

%% multiple remove
function [s, t, weight] = removeNodes(s, t, weight, nodesToRemove)
    % Initialize a logical array to track indices to remove
    indicesToRemove = false(size(s));

    % Find indices of connections involving the specified nodes
    for i = 1:length(nodesToRemove)
        indicesToRemove = indicesToRemove | (s == nodesToRemove(i)) | (t == nodesToRemove(i));
    end
    
    % Remove connections associated with the specified nodes
    s(indicesToRemove) = [];
    t(indicesToRemove) = [];
    weight(indicesToRemove) = [];

    % Update node numbers greater than the removed nodes
    for i = 1:length(nodesToRemove)
        s(s > nodesToRemove(i)) = s(s > nodesToRemove(i)) - 1;
        t(t > nodesToRemove(i)) = t(t > nodesToRemove(i)) - 1;
    end
end

%% remove single element 
function result = removeNthElement(a, n)
    if n >= 1 && n <= numel(a)
        result = [a(1:n-1) a(n+1:end)];
    else
        error('Invalid index. Please enter a valid index within the range of the vector.');
    end
end

%% remove multiple element 
function result = removeNthElements(a, indices)
    if all(indices >= 1) && all(indices <= numel(a))
        result = a;
        result(indices) = [];
    else
        error('Invalid indices. Please enter valid indices within the range of the vector.');
    end
end


function count = countNumbersInFront(array, number)
    % Check if the array is empty
    if isempty(array)
        count = 0;
        return;
    end

    % Find the indices where the array is less than the input number
    indices = find(array < number);

    % Count the number of elements in front of the input number
    count = numel(indices);
end


function [s, t, weight] = increase_weights(s, t, weight, nodesToSelected, increaseFactor)
    % Find indices of connections involving the specified node
    indicesToIncrease = ismember(s, nodesToSelected);
    % Find indices of connections involving the specified node
    indicesToIncrease_t = ismember(t, nodesToSelected);

    % Increase the weights of connections associated with the specified node
    % weight(indicesToIncrease) = weight(indicesToIncrease) * increaseFactor;
    weight(indicesToIncrease) = increaseFactor;
    % weight(indicesToIncrease_t) = weight(indicesToIncrease_t) * increaseFactor;
    weight(indicesToIncrease_t) = increaseFactor;
end