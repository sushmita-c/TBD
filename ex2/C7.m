% Input: cspace -> NxN matrix: cspace(i,j)
%                  == 1 if [q_grid(i); q_grid(j)] is in collision,
%                  == 0 otherwise
% Output: padded_cspace -> NxN matrix: padded_cspace(i,j)
%                          == 1 if cspace(i,j) == 1, or some neighbor of
%                                  cell (i,j) has value 1 in cspace
%                                  (including diagonal neighbors)
%                          == 0 otherwise

function padded_cspace = C7(cspace)

[n,m] = size(cspace);
padded_cspace = cspace;
% imshow(1-padded_cspace');
% set(gca, 'YDir', 'normal');


% Neighboring elements generalized matrix
adj = [[-1 0 1 -1  1 -1 0 1]; [-1 -1 -1 0 0 1 1 1]];

% cell = [1,1];
for i = 1:n
    for j = 1:m
        cell = [i,j]; 
        cell_val = cspace(i,j);
        for k = adj
            adjy = cell(2)+k(2);
            adjx = cell(1)+k(1);
            if adjx<1||adjx>100||adjy<1||adjy>100
                break;
            end
            if cell_val == 1 && cspace(adjx,adjy)==0
                padded_cspace(adjx,adjy) = 3;
            end
        end
    end 
end

% Replace all neighboring 3-value cells with 1 to pad the obstacles
for i = 1:m
    for j =1:n
        if padded_cspace(i,j) == 3
            padded_cspace(i,j) =1;
        end
    end
end

% disp(padded_cspace);
end