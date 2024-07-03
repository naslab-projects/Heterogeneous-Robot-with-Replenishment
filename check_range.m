
a = [0.5 1.0; 2.3 3.8; 4 5];
b = 4.5;

[result, end_time, nth_charging_spot] = checkRange(a, b);

disp(nth_charging_spot);



function [result end_time, nth_charging_spot] = checkRange(A, b)
    % Check if b is in any of the ranges defined by rows of A
    end_time = 0;
    nth_charging_spot = 0;
    % Iterate over each row of A
    for i = 1:size(A, 1)
        currentRange = A(i, :);
        if b >= currentRange(1) && b <= currentRange(2)
            result = true;
            end_time = currentRange(2);
            nth_charging_spot = i;
            return;  % Exit the loop if b is in the range
        end
    end

    % If none of the ranges match, return false
    result = false;
end