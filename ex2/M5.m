% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        path -> Nx4 matrix containing a collision-free path between
%                q_start and q_goal
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: smoothed_path -> Nx4 matrix containing a smoothed version of the
%                          input path, where some unnecessary intermediate
%                          waypoints may have been removed

function smoothed_path = M5(robot, path, link_radius, sphere_centers, sphere_radii)
%disp(path);
[m,n] = size(path);
que = [path(1,:)];
i = 1;
% Check for each vertex if non-collision path available
while i<m-1
    q1 = path(i,:);
    for j =i+1:m
        q2 = path(j,:); 
        coll_bool = check_edge(robot, q1, q2, link_radius, sphere_centers, sphere_radii);
        % if no collision found, update new path point to be added
        % Else continue with loop 1 at at time
        if coll_bool == 0
            q = q2; 
            i = j;
        end
    end
    que = [que;q];
end
smoothed_path= que;
end