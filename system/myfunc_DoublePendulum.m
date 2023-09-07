function [dx,y] = myfunc_DoublePendulum(t,x,u,m1,m2,l1,l2,varargin)
% x = [theta1, theta2, omega1, omega2]

theta1 = x(1);
theta2 = x(2);
omega1 = x(3);
omega2 = x(4);


% m1 = par(1);
% m2 = par(2);
% l1 = par(3);
% l2 = par(4);
g = 9.8;

omega1d = (-g*(2*m1+m2)*sin(theta1)-m2*g*sin(theta1-2*theta2)-2*sin(theta1-theta2)*m2*(omega2^2*l2+omega1^2*l1*cos(theta1-theta2)))/...
    (l1*(2*m1+m2-m2*cos(2*theta1-2*theta2)));
omega2d = (2*sin(theta1-theta2)*(omega1^2*l1*(m1+m2)+g*(m1+m2)*cos(theta1)+omega2^2*l2*m2*cos(theta1-theta2)) )/...
    (l2*(2*m1+m2-m2*cos(2*theta1-2*theta2)));

dx = [omega1;omega2;omega1d;omega2d];
% y = [theta1;theta2;omega1;omega2];
y = [theta1;theta2];


end