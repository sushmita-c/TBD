% Input: robot -> A 2-DOF robot encapsulated in a MATLAB cell with fields:
%                 robot.link1, robot.link2, robot.pivot1, robot.pivot2
%                 See description in hw2_cspace and demo code in C1.
%        q -> 2x1 vector denoting the configuration to convert to polygons
% Output: poly1 -> A polyshape object denoting the 2-D polygon describing
%                  link 1
%         poly2 -> A polyshape object denoting the 2-D polygon describing
%                  link 2
%         pivot1 -> 2x1 vector denoting the 2-D position of the pivot point
%                   of link 1 (frame 1 origin), with respect to base frame
%         pivot2 -> 2x1 vector denoting the 2-D position of the pivot point
%                   of link 2 (frame 2 origin), with respect to base frame

function [poly1, poly2, pivot1, pivot2] = q2poly(robot, q)
   % extract angles 1 and 2 from q
    ang1= q(1);
    ang2= q(2);

    % Determine rotation matrices and find rotated pivot point 2
    R1 = [cos(ang1) -sin(ang1); sin(ang1) cos(ang1)];
    R2 = [cos(ang2) -sin(ang2); sin(ang2) cos(ang2)];
    pivot1 = robot.pivot1;
    pivot2 = R1*robot.pivot2;
    
    % Translate frame origins
    origin1_at0 = pivot1;
    origin2_at0 = origin1_at0 + pivot2; 
    
    % Compute link polygon corners
    for i = 1:4
        robot.link1(:,i) = R1*robot.link1(:,i);
        robot.link2(:,i) = R2*(R1*robot.link2(:,i));
    end
    
    % Compute new link co-ordinates as per frame{0}
    poly1 = robot.link1 + origin1_at0;
    poly2 = robot.link2 + origin2_at0;
end