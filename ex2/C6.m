% Input: robot -> A 2-DOF robot encapsulated in a MATLAB cell with fields:
%                 robot.link1, robot.link2, robot.pivot1, robot.pivot2
%                 See description in hw2_cspace and demo code in C1.
%        obstacles -> 1xN vector of polyshape objects describing N 2-D
%                     polygonal obstacles
%        q_path -> Mx2 matrix containing a collision-free path from
%                  q_start to q_goal. Each row in q_path is a robot
%                  configuration. The first row should be q_start,
%                  the final row should be q_goal.
% Output: num_collisions -> Number of swept-volume collisions encountered
%                           between consecutive configurations in q_path

function num_collisions = C6(robot, obstacles, q_path)
[rows, columns] = size(q_path);
from_vertex = [];
to_vertex = [];
num_collisions = 0;

% Separate q_path into origin config and goal config for each pair
% Calculate swept volume

for i = 1:rows-1
    % Q1 point for calculating swept volume
    from_vertex = [from_vertex;[q_path(i,1), q_path(i,2)]];
    q_f = [from_vertex(i,1); from_vertex(i,2)];
    [x1, y1, ~, ~]= q2poly(robot,q_f);
    
    % Q2 point for calculating swept volume
    to_vertex = [to_vertex;[q_path(i+1,1),q_path(i+1,2)]];
    q_t = [to_vertex(i,1); to_vertex(i,2)];
    [x2, y2, ~, ~]= q2poly(robot,q_t);

    % Calculate swept volume
    sv_arm1 = swept_vol(x1, x2);
    sv_arm2 = swept_vol(y1, y2);
    comb_sv = union(sv_arm1, sv_arm2);
    
    % Check overlap with each obstacle and plot if any
    for j = obstacles
        if intersect(j, comb_sv).NumRegions>0
            plot(sv_arm1, 'FaceColor', 'b');
            plot(sv_arm2, 'FaceColor', 'r');
            % Increase number of collisions by 1;
            num_collisions = num_collisions + 1;
            break;
        end
    end
end
end
    

function swept_volume = swept_vol(from, to)
    a = from';
    b = to';
    points = [a; b];
    k = convhull(points);
    swept_volume = polyshape(points(k,1), points(k,2));
end