% Input: q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        num_samples -> Integer denoting number of samples to sample
% Output: qs -> num_samples x 4 matrix of joint angles,
%               all within joint limits

function qs = M1(q_min, q_max, num_samples)
% r = a + (b-a).*rand(100,1);

q_1 = linspace(q_min(1),q_max(1),num_samples);
%disp(q_1);
q_2 = linspace(q_min(2),q_max(2),num_samples);
%disp(q_2);
q_3 = linspace(q_min(3),q_max(3),num_samples);
%disp(q_3);
q_4 = linspace(q_min(4),q_max(4),num_samples);
%disp(q_4);

qt = [q_1; q_2; q_3; q_4;];
qs=qt';
%disp(size(qs));

end