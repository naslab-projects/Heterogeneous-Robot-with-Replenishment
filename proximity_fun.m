
% Example usage:
A = [9.2175, 27.1546, 45.1546, 9.7726, 27.7726, 9.7749, 27.7749, 35, 35.5, 99];
thresholdValue = 1.0;  % Adjust this threshold as needed

B = proximity(A, thresholdValue);
disp(B);

function B = proximity(A, threshold)
    groups = containers.Map('KeyType', 'double', 'ValueType', 'any');
    groupCount = 1;

    for i = 1:length(A)
        assigned = false;

        keys = groups.keys;
        for j = 1:length(keys)
            group = groups(keys{j});
            if any(abs(A(i) - group) <= threshold)
                groups(keys{j}) = [group, A(i)];
                assigned = true;
                B(i) = keys{j};  % Assign the group number to the current element
                break;
            end
        end

        if ~assigned
            groups(groupCount) = A(i);
            B(i) = groupCount;  % Assign the new group number to the current element
            groupCount = groupCount + 1;
        end
    end
end
