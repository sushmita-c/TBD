% Input: distances -> NxN matrix containing the distance transform from
%                      the goal configuration
%                      == 0 if cell is unreachable
%                      == 1 if cell is an obstacle
%                      == 2 if cell is the goal
%                      >  2 otherwise
%        q_grid -> 1xN vector of angles between 0 and 2*pi, discretizing
%                  each dimension of configuration space
%        q_start -> 2x1 vector denoting the start configuration
% Output: path -> Mx2 matrix containing a collision-free path from q_start
%                 to q_goal (as computed in C3, embedded in distances).
%                 The entries of path should be grid cell indices, i.e.,
%                 integers between 1 and N. The first row should be the
%                 grid cell containing q_start, the final row should be
%                 the grid cell containing q_goal.

function path = C4(distances, q_grid, q_start)
%disp(distances);
L= [];
% Check if goal vertex is within range - Validation
if q_start(1)<q_grid(1)||q_start(1)>q_grid(100)||q_start(2)<q_grid(1)||q_start(2)>q_grid(100)
    disp("Robot goal is outside map.");
end

% Find corresponding 100x100 value for q1 q2 on cspace
[idx_x,val1] = closest(q_grid,q_start(1));
[idx_y,val2] = closest(q_grid,q_start(2));

% Adjacency matrix
adj = [[-1 0 1 -1  1 -1 0 1]; [-1 -1 -1 0 0 1 1 1]];
%adj = [[-1 0 1 -1  1 -1 0 1]; [1 1 1 0 0 -1 -1 -1]];
%diag = [[-1 1 -1 1];[1 1 -1 -1 ]; ];

L = [idx_x,idx_y];
current_value = distances(L(1),L(2));
path = [idx_x,idx_y];
if current_value==2
    path = current;
end
 
while(current_value ~= 2)
    cell = L(1,:);
    L(1,:) = [] ;
    for j = adj 
        j_x = cell(1) + j(1);
        j_y = cell(2) + j(2);
        if distances(j_x,j_y)==current_value-1&&current_value~=2
            path = [path;[j_x,j_y]];
            L = [L;[j_x,j_y]];
            current_value = distances(j_x,j_y);
        end
    end
end
end

% Find closest value on cspace matrix to g_goal coordinates
function [idx, val] = closest(q_grid,val)
tmp = abs(q_grid - val);
[~, idx] = min(tmp);
val = q_grid(idx);
end