% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        samples -> num_samples x 4 matrix, vertices in the roadmap
%        adjacency -> num_samples x num_samples matrix, the weighted
%                     adjacency matrix denoting edges in the roadmap
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

function [path, path_found] = M3(robot, samples, adjacency, q_start, q_goal, link_radius, sphere_centers, sphere_radii)
    
    % V is our sampled vertexes matrix
    V = samples;
    % E is our adjacency matrix
    E = adjacency;
    
   
    % For the start vertex, check collision possibilty
    collision_bool_val_s = check_collision(robot, q_start, link_radius, sphere_centers, sphere_radii);
    
    % if Start vertex not in collision find distances & add to Vertex list
    if collision_bool_val_s == 0
        A = q_start;
        % Calculate distances to find nearest neighbors
        for i=1:length(V)
            % Each distance between each vertex B and the Start vertex A
            B = V(i,:);
            dist_from_q_start(1,i) = sqrt((B(4)-A(4))^2+ (B(3)-A(3))^2+(B(2)-A(2))^2+(B(1)-A(1))^2);
        end
%         disp(dist_from_q_start);
%          V = [V;q_start];
%          disp(length(V));
    end
    
    % For the goal vertex, check collision possibilty
    collision_bool_val_g = check_collision(robot, q_goal, link_radius, sphere_centers, sphere_radii);
    
    % if Goal vertex not in collision add to Vertex list
    if collision_bool_val_g == 0
        A = q_goal;
        % Calculate distances to find nearest neighbors
        for i=1:length(V)
            % Each distance between each vertex B and the Start vertex A
            B = V(i,:);
            dist_from_q_goal(1,i) = sqrt((B(4)-A(4))^2+ (B(3)-A(3))^2+(B(2)-A(2))^2+(B(1)-A(1))^2);
        end
%         disp(dist_from_q_goal);
%         V = [V;q_goal];
%         disp(length(V));
    end
%     disp(V);
V = [V;q_start;q_goal];

% Sort by asc distance and get nearest neighbors matrix  
[sorted_dist_q_start, nearest_neighbor_qs_idx] = sort(dist_from_q_start,2);
[sorted_dist_q_goal, nearest_neighbor_qg_idx] = sort(dist_from_q_goal,2);
% disp(nearest_neighbor_qs_idx);
% disp(nearest_neighbor_qg_idx);

%%%%%%%%%%%%%%%%%% CHECK FOR CONNECTIONS B/W NEIGHBORS %%%%%%%%%%%%%%%%%%%

% Create new matrix two columns greater than current adj matrix
% This accomodates for the start and goal vertexes distances
new_E = zeros(length(V), length(V));
new_E(1:end-2,1:end-2)=E;

% Check connection for each nearest neighbor to q_start
for j=1:length(nearest_neighbor_qs_idx)
    vertex_no = nearest_neighbor_qs_idx(1,j);
    neighbor_node = V(vertex_no,:);
    % Check edge existence
    edge_bool_val_s = check_edge(robot, q_start, neighbor_node, link_radius, sphere_centers, sphere_radii);
    
    % if edge is collision free add distance to adj matrix
    if edge_bool_val_s == 0
        d = sqrt((neighbor_node(4)-q_start(4))^2+ (neighbor_node(3)-q_start(3))^2+(neighbor_node(2)-q_start(2))^2+(neighbor_node(1)-q_start(1))^2);
        new_E(length(V)-1,vertex_no) = d;
        new_E(vertex_no, length(V)-1) = d;
    end
end

% Check connection for each nearest neighbor to q_goal
for j=1:length(nearest_neighbor_qg_idx)
    vertex_no = nearest_neighbor_qg_idx(1,j);
    neighbor_node = V(vertex_no,:);
    % Check edge existence
    edge_bool_val_g = check_edge(robot, q_goal, neighbor_node, link_radius, sphere_centers, sphere_radii);
    
    % if edge is collision free add distance to adj matrix
    if edge_bool_val_g == 0
        d = sqrt((neighbor_node(4)-q_goal(4))^2+ (neighbor_node(3)-q_goal(3))^2+(neighbor_node(2)-q_goal(2))^2+(neighbor_node(1)-q_goal(1))^2);
        new_E(length(V),vertex_no) = d;
        new_E(vertex_no, length(V)) = d;
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%% FIND IF PATH EXISTS %%%%%%%%%%%%%%%%%%%%%%%%%%
% Form graph
% gr = [zeros(102,102), new_E; new_E', zeros(102,102)];     
adjacency = new_E;
G = digraph(adjacency);

% %  SYMMETRIC CHECK
% for i = 1 : 1
%     for j = 1 : 1
%         fprintf(['The matrix is a' repmat('\b',isequal(adjacency,adjacency.')) 'symmetric!\n']);
%     end
% end

% Find vertex number for q_start
for i=1:length(V)
    if V(i,:) == q_start
        start_vertex = i;
    end
end

% Find vertex number for q_start
for i=1:length(V)
    if V(i,:) == q_goal
        goal_vertex = i;
    end
end

% Find shortest path if possible
v_path = shortestpath(G, start_vertex, goal_vertex);

path = [];

% Translate from vertex path to config path
for i = 1:length(v_path)
    path = [path;V(v_path(i),:)];
end
    
% path = shortestpath(G, 7, 22);
% plot(graph(adjacency));

path_found = 0;
% If path is found, return 1 else 0
if v_path
    path_found = 1;
end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% END %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

