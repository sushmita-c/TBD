% Output: q -> 1x4 vector of joint angles

function q = M0()
    q = [0 -pi/4 0 -pi/4];
end

% Familiarize yourself with the provided code in ex2 motion.m and the robot that has been defined.
% Use the code in M0.m to visualize the robot in various configurations. The code in ex2 motion.m that calls M0
% also checks whether the configuration is within joint limits, and whether the configuration is in collision.
% Look at the code in check collision.m and check edge.m. Explain what each function is doing to check
% for collisions, for individual configurations and for configuration-space line segments respectively. Identify and
% describe two potential issues in the collision-checking algorithm.

% Check_collision function takes the point 'q' and  provided by user and
% calculates each individual joint's configuration location in the workspace. 
% We then calculate the distance of each joint point in 'q' and
% divide it into a configuration space line from 1 to 2 and 2
% to 3 (points in space for the joints). This is represented by x12 and x23
% config-space lines. The collision checking occurs in the for loop where
% 'resolution' number of points along x12 and x23 are checked against the
% points enclosed by the sphere (surface points of the sphere). If any
% lie within the sphere points (i.e. (link_radius + sphere_radii(i)).^2)) then
% the collision function will be true and the point 'q' will be in
% collision with the obstacles (in our case, the sphere).
%
% Check _edge functions takes the two user input points q_start and q_end,
% calculates the line segment (configuration line space) between them and
% discretizes points in this line segment (linspace). These points 
% ('resolution' number of points) are now stored in 'configs' and
% each point in config is passed into the 'check_collision function' to see
% if it collides with any obstacles (sphere). If the points in 'configs'
% are all collision free, then the edge is also said to be collision free.

% One major potential issue with the collision checking algorithm is that
% the resolution is always 11 because the number of inputs are always less than
% 6. This allows the line space to be divided into 11 parts. However, if
% there are point sized (or slightly bigger, but still relatively small) obstacles in the
% workspace, the possibility of miscalculating a collision becomes very
% high. 
%
% Another potential issue is that while the robot's arm (in length) has
% been taken into consideration, the part at the joint that connects the
% two links together is not taken into consideration and may cause a
% problem when this simulation is run on a real life robotic arm as it will
% end up potentially knocking over objects that it was supposed to move
% around. 
