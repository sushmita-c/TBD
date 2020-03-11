% Input: robot -> A 2-DOF robot encapsulated in a MATLAB cell with fields:
%                 robot.link1, robot.link2, robot.pivot1, robot.pivot2
%                 See description in hw2_cspace and demo code in C1.
%        obstacles -> 1xN vector of polyshape objects describing N 2-D
%                     polygonal obstacles
%        q_grid -> 1xN vector of angles between 0 and 2*pi, discretizing
%                  each dimension of configuration space
% Output: cspace -> NxN matrix: cspace(i,j)
%                   == 1 if [q_grid(i); q_grid(j)] is in collision,
%                   == 0 otherwise

function cspace = C2(robot, obstacles, q_grid)
plot_obstacles(obstacles);
[x,y] = size(q_grid);
for i = 1:y
    for j = 1:y
        % Separate q_grid into two radian angles for input to robot joints
        q = [q_grid(i);q_grid(j)];
        % Run plot for  q = [q1 q2] combination
        [poly1, poly2, pivot1, pivot2] = q2poly(robot, q);

        % Intersection points for first link of arm (if any)
        a = intersect(obstacles,polyshape(poly1(1,:), poly1(2,:)));
        % Intersection points for second link of arm (if any)
        b = intersect(obstacles,polyshape(poly2(1,:), poly2(2,:)));

        % Lengths of a and b
        [c,sa] = size(a);
        [d,sb] = size(b);
        for k=1:sa
            for n=1:sb
                if (a(k).NumRegions>0)||(b(n).NumRegions>0)
                    cspace(i,j) = 1;
                end
            end
        end
    end
end
end