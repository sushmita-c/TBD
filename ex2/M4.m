% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        q_start -> 1x4 vector denoting the start configuration
%        q_goal -> 1x4 vector denoting the goal configuration
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: path -> Nx4 matrix containing a collision-free path between
%                 q_start and q_goal, if a path is found. The first row
%                 should be q_start, the final row should be q_goal.
%         path_found -> Boolean denoting whether a path was found

function [path, path_found] = M4(robot, q_min, q_max, q_start, q_goal, link_radius, sphere_centers, sphere_radii)

% stepsize is alpha beta is q_target checker
alpha = 0.25;
beta = 0.10;

% Initialize V with q_start and E as an empty matrix
V = [q_start];
E = [];

% [n,m]= size(V);
i = 1;
while i<=350
    r = (1-0).*rand(1,1) + 0;
%     disp(r);
    if r<beta
        q_target = q_goal;
    else 
        % Sample random angle
        q_1 = q_min(1) + (q_max(1)-q_min(1)).*(rand(1,1));
        q_2 = q_min(2) + (q_max(2)-q_min(2)).*(rand(1, 1));
        q_3 = q_min(3) + (q_max(3)-q_min(3)).*(rand(1, 1));
        q_4 = q_min(4) + (q_max(4)-q_min(4)).*(rand(1,1));

        q_target = [q_1, q_2, q_3, q_4];
    end    
    
    % Find nearest neighbor to q_target
    [matrix_n, nearest_neighbor] = find_nn(V,q_target);
    q_near = V(nearest_neighbor,:);
    
    q_diff = q_target - q_near;
    abs_q_diff = sqrt(q_diff(1)^2 + q_diff(2)^2+q_diff(3)^2+q_diff(4)^2);
    
    % Calculate q_new within stepsize
    if alpha>abs_q_diff
        q_new = q_target;
    else 
        q_new = q_near + ((alpha/abs_q_diff)*(q_diff));
    end
    
    %if q_new(1)<=q_max(1)&&q_new(1)>=q_min(1)&&q_new(2)<=q_max(2)&&q_new(2)>=q_min(2)&&q_new(3)<=q_max(3)&&q_new(3)>=q_min(3)&&q_new(4)<=q_max(4)&&q_new(4)>=q_min(4)
        
    % Check if q_new is collision free in sample space
    check_collision_q_new = check_collision(robot, q_new, link_radius, sphere_centers, sphere_radii);
    if check_collision_q_new == 0
        check_edge_collision = check_edge(robot, q_near, q_new, link_radius, sphere_centers, sphere_radii);
        if check_edge_collision == 0
            V = [V; q_new];
            E = [E; q_near, q_new];
        end
        %end 
    end
    i=i+1;
%     else
%         i=i+1;
%     end
end    

%%%%%%%%%%%%%%%%%%% ADD GOAL TO VERTEX  & ADJ MATRIX %%%%%%%%%%%%%%%%%%%%
V = [V; q_goal];
[matrix_n, nn_to_goal] = find_nn(V,q_goal);
[a,b] = size(matrix_n);
for counter = 1:b
    nn_to_goal = matrix_n(1,counter);
    ang = V(nn_to_goal, :);
    check_edge_collision = check_edge(robot, q_goal, ang, link_radius, sphere_centers, sphere_radii);
    di = sqrt((q_goal(4)-ang(4))^2+(q_goal(3)-ang(3))^2+(q_goal(2)-ang(2))^2+(q_goal(1)-ang(1))^2);
    if check_edge_collision == 0 && di<=alpha
        E=[E; ang, q_goal];
        break;
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%% ADJACENCY MATRIX %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[no_of_vertices,y_V] = size(V);
[pair_of_edges,y_E] = size(E);

adjacency = zeros(no_of_vertices, no_of_vertices);

for edge_i = 1:pair_of_edges
    q1 = E(edge_i,1:4);
    q2 = E(edge_i,5:8);
    for vertex_i = 1:length(V)
        if q1 == V(vertex_i,:)
            node1 = vertex_i;
%             disp(node1);
        end
        if q2 == V(vertex_i,:)
            node2 = vertex_i;
%             disp(node2);
        end
    end
    d = sqrt((q2(4)-q1(4))^2+ (q2(3)-q1(3))^2+(q2(2)-q1(2))^2+(q2(1)-q1(1))^2);
    adjacency(node1, node2) = d;
    adjacency(node2, node1) = d;
end

%%%%%%%%%%%%%%%%%%%%%%% FORM GRAPH AND FIND PATH %%%%%%%%%%%%%%%%%%%%%%%%%
path_found = 0;
path = [];

G = graph(adjacency);
v_path = shortestpath(G, 1, length(V));

% Translate from vertex path to config path
for i = 1:length(v_path)
    path = [path;V(v_path(i),:)];
end

if v_path
    path_found = 1;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% END %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end




% Function to find nearest neighbor of q_target in V 
% Input: V -> A Nx4 matrix of joint configurations of vertexes
%        q_target -> 1x4 vector of config for a target joint
% Output: nearest_neighbor -> 1x4 vector of config for nearest joint

function [matrix_n, nearest_neighbor] = find_nn(V, q_target)
% disp(length(V));
[m,n]= size(V);
% disp(size(V));  %% 1x4 matrix
    for i=1:m
        A = q_target;
        B = V(i,:);
        dist(1,i) = sqrt((B(4)-A(4))^2+ (B(3)-A(3))^2+(B(2)-A(2))^2+(B(1)-A(1))^2);
    end
    % Sort neighbors by increasing distance
    [sorted_dist, neighbor_matrix] = sort(dist,2);
    matrix_n = neighbor_matrix;
    nearest_neighbor = neighbor_matrix(1);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% END %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
