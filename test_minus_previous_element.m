A = [2 2 1 2 1 2];
value_to_find = A(4);

% Find the indices of the same value before the 4th element
indices = find(A(1:3) == value_to_find);

disp(indices(end));