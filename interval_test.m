A = [1 5.6; 7 10];
B = [2 5; 11 12];
% A = [];
% B = [];
%C = [9 10];
C = [4.5 6.9];
[updatedA, updatedB, space_remain] = insertIntervals(A, B, C);

disp('Updated A:');
disp(updatedA);

disp('Updated B:');
disp(updatedB);
space_remain

function [updatedA, updatedB, space_remain] = insertIntervals(A, B, intervals)
    % Sort intervals based on start times
    intervals = sortrows(intervals, 1);

    % Initialize updated matrices
    updatedA = A;
    updatedB = B;
    space_remain = 8;
    % Iterate through intervals and insert into the appropriate matrix
    for i = 1:size(intervals, 1)
        startTime = intervals(i, 1);
        endTime = intervals(i, 2);

        % Determine which matrix to insert into
        if isempty(updatedA) || ~isIntervalOccupied(updatedA, startTime, endTime)
            % Insert into matrix A
            updatedA = insertIntoMatrix(updatedA, [startTime endTime]);
        elseif isempty(updatedB) || ~isIntervalOccupied(updatedB, startTime, endTime)
            % Insert into matrix B
            updatedB = insertIntoMatrix(updatedB, [startTime endTime]);
        else
            space_remain = 0;
            %error('Cannot insert interval, both matrices are occupied during the specified time.');
        end
    end
end

function occupied = isIntervalOccupied(matrix, startTime, endTime)
    % Check if the interval is occupied in the given matrix
    occupied = any(startTime >= matrix(:, 1) & startTime <= matrix(:, 2)) || ...
               any(endTime >= matrix(:, 1) & endTime <= matrix(:, 2));
end

function updatedMatrix = insertIntoMatrix(matrix, interval)
    % Find the appropriate position to insert the interval
    inserted = false;
    for j = 1:size(matrix, 1)
        if interval(1) < matrix(j, 1)
            matrix = [matrix(1:j-1, :); interval; matrix(j:end, :)];
            inserted = true;
            break;
        end
    end

    % If the interval is greater than all existing end times, append it
    if ~inserted
        matrix = [matrix; interval];
    end

    updatedMatrix = matrix;
end


% function [updatedA, updatedB] = insertIntervals(A, B, intervals)
%     % Sort intervals based on start times
%     intervals = sortrows(intervals, 1);
% 
%     % Initialize updated matrices
%     updatedA = A;
%     updatedB = B;
% 
%     % Iterate through intervals and insert into the appropriate matrix
%     for i = 1:size(intervals, 1)
%         startTime = intervals(i, 1);
%         endTime = intervals(i, 2);
% 
%         % Determine which matrix to insert into
%         if isempty(updatedA) || startTime < updatedA(end, 1)
%             % Insert into matrix A
%             updatedA = insertIntoMatrix(updatedA, [startTime endTime]);
%         else
%             % Insert into matrix B
%             updatedB = insertIntoMatrix(updatedB, [startTime endTime]);
%         end
%     end
% end
% 
% function updatedMatrix = insertIntoMatrix(matrix, interval)
%     % Find the appropriate position to insert the interval
%     inserted = false;
%     for j = 1:size(matrix, 1)
%         if interval(1) < matrix(j, 1)
%             matrix = [matrix(1:j-1, :); interval; matrix(j:end, :)];
%             inserted = true;
%             break;
%         end
%     end
% 
%     % If the interval is greater than all existing end times, append it
%     if ~inserted
%         matrix = [matrix; interval];
%     end
% 
%     updatedMatrix = matrix;
% end