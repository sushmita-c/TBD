% input: f1 -> an 9-joint robot encoded as a SerialLink class for one
%              finger
%        f2 -> an 9-joint robot encoded as a SerialLink class for one
%              finger
%        qInit -> 1x11 vector denoting current joint configuration.
%                 First seven joints are the arm joints. Joints 8,9 are
%                 finger joints for f1. Joints 10,11 are finger joints
%                 for f2.
%        f1Target, f2Target -> 3x1 vectors denoting the target positions
%                              each of the two fingers.
% output: q -> 1x11 vector of joint angles that cause the fingers to
%              reach the desired positions simultaneously.
%              (orientation is to be ignored)

function q = Q5(f1, f2, qInit, f1Target, f2Target)
    q = qInit;
    K = 0;
    
    % K is some sufficiently large sample
    while K < 500
        % Joint config for f1 and f2 - first seven are common, 8,9 for f1
        % and 10,11 for f2
        q1 = q(:, (1 : 9));
        q2 = q(:, [1 : 7, 10 : 11]);
       
        % Jacobian for q1 and q2
        J = f1.jacob0(q1, 'trans') + f2.jacob0(q2,'trans');

        % Forward kinematics map for f1 and f2
        T1 = f1.fkine(q1);
        T2 = f2.fkine(q2);
        
        % f1, f2 translation
        Dx1 = f1Target - transpose(transl(T1));
        Dx2 = f2Target - transpose(transl(T2));
        
        % f1, f2 Configurations
        Dq1 = pinv(J) * Dx1;
        Dq2 = pinv(J) * Dx2;
        
        Dq_s = vertcat((Dq1(1 : 7, :) + Dq2(1 : 7, :)) / 2, Dq1(8 : 9, :), Dq2(8 : 9, :));
        q = q + transpose(Dq_s);
        K = K + 1;
    end
end