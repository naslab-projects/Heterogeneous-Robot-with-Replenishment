a = [0 0 0 1 0 0 1];
b = [1 0 3 5 7 8 6];

% Find indices where a is equal to 0
zero_indices = find(a == 0);

% Select the maximum value's index from b at those positions
max_index = zero_indices(b(zero_indices) == min(b(zero_indices)));

disp("Index of maximum value in b where a is 0: " + max_index);