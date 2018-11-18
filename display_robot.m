%
% Author Patric Jensfelt
%
function [h] = display_robot(x,y,a,c,drawDirectionVector)

if nargin < 4
    c = 'r';
end

if nargin < 5
    drawDirectionVector = 0;
end


% Length and whidth of the robot
Length = 0.5;
Width = 0.5;

X = [-0.5*Length 0.5*Length 0.5*Length -0.5*Length -0.5*Length;
    -0.5*Width -0.5*Width 0.5*Width 0.5*Width -0.5*Width];

R = [cos(a) -sin(a);sin(a) cos(a)];

X = R*X;

h = plot(x+X(1,:),y+X(2,:),c,'LineWidth',2);
if drawDirectionVector
    h = [h plot([x+[0 2.5*cos(a)]],[y+[0 2.5*sin(a)]],c,'LineWidth',2)];
end