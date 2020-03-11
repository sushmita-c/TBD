% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        num_samples -> Integer denoting number of samples in PRM
%        num_neighbors -> Integer denoting number of closest neighbors to
%                         consider in PRM
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: samples -> num_samples x 4 matrix, sampled configurations in the
%                    roadmap (vertices)
%         adjacency -> num_samples x num_samples matrix, the weighted
%                      adjacency matrix denoting edges between roadmap
%                      vertices. adjacency(i,j) == 0 if there is no edge
%                      between vertex i and j; otherwise, its value is the
%                      weight (distance) between the two vertices. For an
%                      undirected graph, the adjacency matrix should be
%                      symmetric: adjacency(i,j) == adjacency(j,i)

function [samples, adjacency] = M2(robot, q_min, q_max, num_samples, num_neighbors, link_radius, sphere_centers, sphere_radii)
% disp(q_min); % [-1.5708   -3.1416         0   -3.1416]
% disp(q_max); % [1.5708         0         0         0]
% disp(num_samples); % 100
% disp(num_neighbors); % 10
% disp(link_radius); % 0.0300
% disp(sphere_centers); % [0.5000         0         0]
% disp(sphere_radii); % 0.2500

n = num_samples;
num_n = num_neighbors;
samples = [];

i=length(samples)+1;


% Populate 'samples' with num_samples number of collision free configurations
while i<=n
   % Get a random joint configuration and store it in variable ang
   q_1 = q_min(1) + (q_max(1)-q_min(1)).*(rand(1,1));
   q_2 = q_min(2) + (q_max(2)-q_min(2)).*(rand(1, 1));
   q_3 = q_min(3) + (q_max(3)-q_min(3)).*(rand(1, 1));
   q_4 = q_min(4) + (q_max(4)-q_min(4)).*(rand(1,1));
%    ang =[q(i),q(i,2),q(i,3),q(i,4)];
   ang =[q_1,q_2,q_3,q_4];
    % check to see if there is a collision of that joint configuration with
    % any obstacle
   bool_val = check_collision(robot, ang, link_radius, sphere_centers, sphere_radii);
   % if there is no collision, store configuration in samples matrix
   if bool_val==0
       samples = [samples;ang];
   end
   % Check samples length at 100
   if length(samples) == num_samples
       n = 100;
   else
       n = n+1;
   end
   i=i+1;
end

V = samples;

% find k nearest neighbors
for j = 1:length(V)
    for k = 1:length(V)
        % Take into account first vertex
        A =  V(j,:);
        % Take into account every other vertex 
         B = V(k,:);
         dist(j,k) = sqrt((B(4)-A(4))^2+ (B(3)-A(3))^2+(B(2)-A(2))^2+(B(1)-A(1))^2);
    end
end
% disp(dist);

% Sort matrix by ascending distance and get a corresponding matrix 
[sorted_dist, nn_mat] = sort(dist,2);


nn_mat(:,1) = [];
%disp(size(nn_mat)); %[100 , 99]
knn = nn_mat(1:num_samples,1:num_n);
[a, b] = size(knn);
% disp(a);
% disp(b);

% Create adjacency matrix full of zeroes
adjacency = zeros(num_samples,num_samples);

for i =1:a
    curr_node =  V(i,:);
    for j=1:b
        vertex_no = knn(i,j);
% %         disp(vertex_no);
        neighbor_node = V(vertex_no,:);
% %         disp(neighbor_node)
        edge_bool_val = check_edge(robot, curr_node, neighbor_node, link_radius, sphere_centers, sphere_radii);
%         if collision does not exist, add distance
        if edge_bool_val == 0
            d = sqrt((neighbor_node(4)-curr_node(4))^2+ (neighbor_node(3)-curr_node(3))^2+(neighbor_node(2)-curr_node(2))^2+(neighbor_node(1)-curr_node(1))^2);
            adjacency(i,vertex_no) = d;
            adjacency(vertex_no,i) = d;
        end
    end
end
% 
% %disp(adjacency);
plot(graph(adjacency));
% for i = 1 : 1
%     for j = 1 : 1
%         fprintf(['The matrix is a' repmat('\b',isequal(adjacency,adjacency.')) 'symmetric!\n']);
%     end
% end

end