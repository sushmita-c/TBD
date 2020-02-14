% input: f -> an 9-joint robot encoded as a SerialLink class
%        position -> 3x1 vector denoting the target position to move to
% output: q -> 1x9 vectocircler of joint angles that cause the end
%              effector position to reach <position>
%              (orientation is to be ignored)

function q = Q1(f, position)
% Homogenous transform representing translation of position 
    T = transl(position);
    %disp(position);
    %disp(T);
% inverse kinematics using iterative numerical method 
    q = f.ikine(T, 'mask',[1,1,1,0,0,0]); 
    %disp(q);
end