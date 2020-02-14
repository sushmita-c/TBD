% input: f -> a 9-joint robot encoded as a SerialLink class
%        qInit -> 1x9 vector denoting current joint configuration
%        circle -> 3xn matrix of Cartesian positions that describe the
%                  circle
%        velocity -> scalar denoting desired velocity of end effector
% output: traj -> nx9 matrix that denotes arm trajectory. Each row denotes
%                 a joint configuration. The first row is the first
%                 configuration. The last is the last configuration in the
%                 trajectory. The end effector should trace a circle in the
%                 workspace, as specified by the input argument circle.
%                 (orientation is to be ignored)

% I'm doing my best so wherever it takes me is better than where I started.
function traj = Q4(f, qInit, circle, velocity)
traj = [];
q = qInit;
a = size(circle);
circ_size = a(1,2);
%disp(circ_size);

for i = 1:circ_size
    column_i = circle(:,i);
    T = transl(column_i);
    r = f.ikine(T, 'mask',[1,1,1,0,0,0]); 
    traj2 = Q3(f,q,column_i,0.01,velocity);
    traj = [traj;traj2];
    q = r;
end
end

% Alternate way of traversing the circle (fixed velocity)
%function traj = Q4(f, qInit, circle, velocity)
%q = qInit;
%traj =[];
%for i = 1:15
%    column_i = circle(:,i);
%    T = transl(column_i);
%    r = f.ikine(T, 'mask',[1,1,1,0,0,0]); 
%    t = transpose([0:0.05:2]);
%    traj = [traj;jtraj(q,r, t )];
%    q = r;
%end
%end