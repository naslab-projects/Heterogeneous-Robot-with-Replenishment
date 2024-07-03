clc;
clear all;
close all;
% Given matrix a
a = [
    1	1;
    1	2;
    2   1;
    % ... (other elements)
    14	14
];

% Indices of elements to be removed
elements_to_remove = [
    2, 1;
    5, 5;
    % ... (other elements to be removed)
    12, 10
];

% Check if the indices are within the range of the matrix
valid_indices = all(elements_to_remove > 0 & elements_to_remove <= size(a), 2);

% Convert the valid indices to linear indices
indices_to_remove = sub2ind(size(a), elements_to_remove(valid_indices, 1), elements_to_remove(valid_indices, 2));

% Remove the specified elements
a(indices_to_remove, :) = [];

% Display the modified matrix
disp(a);