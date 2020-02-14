% input: f -> a 9-joint robot encoded as a SerialLink class
%        qInit -> 1x9 vector denoting current joint configuration
%        posGoal -> 3x1 vector denoting the target position to move to
% output: q -> 1x9 vector of joint angles that cause the end
%              effector position to reach <position>
%              (orientation is to be ignored)

function q = Q2(f, qInit, posGoal)
% Useful functions: SerialLink.fkine, SerialLink.jacob0, pinv
    q = qInit;
    K = 0;
    while K<500
        x = transpose(transl(f.fkine(q)));
        Dx = posGoal - x;
        
        J = f.jacob0(q, 'trans');
        Dq = pinv(J) * Dx;
        
        q = q + transpose(Dq);
        K = K + 1;
    end
    %disp(q);
end