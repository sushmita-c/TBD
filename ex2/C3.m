 % Input: cspace -> NxN matrix: cspace(i,j)
%                   == 1 if [q_grid(i); q_grid(j)] is in collision,
%                   == 0 otherwise
%        q_grid -> 1xN vector of angles between 0 and 2*pi, discretizing
%                  each dimension of configuration space
%        q_goal -> 2x1 vector denoting the goal configuration
% Output: distances -> NxN matrix containing the distance transform from
%                      the goal configuration
%                      == 0 if cell is unreachable
%                      == 1 if cell is an obstacle
%                      == 2 if cell is the goal
%                      >  2 otherwise

function distances = C3(cspace, q_grid, q_goal)

% Copy cspace into distances
distances = cspace;

% Check if q_goal is within range - Validation
if q_goal(1)<q_grid(1)||q_goal(1)>q_grid(100)||q_goal(2)<q_grid(1)||q_goal(2)>q_grid(100)
    disp("Robot goal is outside map.");
end

[m,n] = size(cspace);

% Find corresponding 100x100 value for q1 q2 on cspace
[idx_x,val1] = closest(q_grid,q_goal(1));
[idx_y,val2] = closest(q_grid,q_goal(2));

% Plot q_goal on map
distances(idx_x,idx_y) = 2;

% Copy values into an array L
L =[idx_x, idx_y];
% Find size of L - x dimension
[L_size, ~] = size(L);

% Adjacency matrix
adj = [[-1 0 1 -1  1 -1 0 1]; [1 1 1 0 0 -1 -1 -1]];

while (L_size>0) 
    cell = L(1,:);
    cell_value = distances(cell(1),cell(2));
    L(1,:) = [] ;
    
    % For all adjacent cells assign a number based on condition
    for j = adj
        j_x = cell(1) + j(1);
        j_y = cell(2) + j(2);
        if (j_x>=1)&&(j_x<=n)&&(j_y>=1)&&(j_y<=n)&&distances(j_x,j_y) == 0
            L = [L;[j_x,j_y]];
            distances(j_x,j_y) = cell_value + 1;
        end
    end
    [L_size, ~] = size(L);
end
%disp(distances);
end


% Find closest value on cspace matrix to g_goal coordinates
function [idx, val] = closest(q_grid,val)
tmp = abs(q_grid - val);
[~, idx] = min(tmp);
val = q_grid(idx);
end