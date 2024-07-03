A = [1 14; 1 13; 1 12; 1 1; 2 1; 2 2; 2 3; 2 14; 3 14; 3 13];
B = [2 1];

% Check for rows in A that match vector B
indices = ismember(A, B, 'rows');

% Find the row indices where the match occurs
row_indices = find(indices);

% Display the result
disp('Row indices where B is found in A:');
disp(row_indices);