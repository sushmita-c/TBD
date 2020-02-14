% input: f -> a 9-joint robot encoded as a SerialLink class
%        qInit -> 1x9 vector denoting current joint configuration
%        posGoal -> 3x1 vector describing goal position
%        epsilon -> scalar denoting how close end effector must be before
%                   loop terminates
%        velocity -> scalar denoting desired velocity of end effector
% output: traj -> nx9 matrix that denotes arm trajectory. Each row denotes
%                 a joint configuration. The first row is the first
%                 configuration. The last is the last configuration in the
%                 trajectory.

function traj = Q3(f, qInit, posGoal, epsilon, velocity)
% Hint: You may be tempted to interpolate the straight line end-effector trajectory and make multiple calls to
% Q2, but a preferred (more efficient) approach is to modify the Jacobian pseudoinverse iteration itself.
    
q = qInit;
K = 1;
traj = [];
    while K >= epsilon
        x = transpose(transl(f.fkine(q)));
        Dx = posGoal - x;

        J = f.jacob0(q, 'trans');
        Dq = pinv(J) * velocity * Dx;
    
        q = q + transpose(Dq);

        traj = [traj;q];
        K = norm(Dx);
    end
    %disp(traj);
end