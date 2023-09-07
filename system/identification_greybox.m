% nonlinear greybox model 

clear all 
close all
clc
%% generate system data

tspan = [0:1e-3:5];
m1 = 2;
m2 = 1.5;
L1=2;
L2=3;
g = 9.8;
x0 = [pi/2;pi/2;0;0];



[t,x] = ode45(@(t,x)double_pendulum_system(x,L1,L2,m1,m2,g), tspan, x0);

% add sensing noise
x_res = x + 0.3*rand(size(x));
z = iddata(x_res(:,1:2), zeros(length(tspan),2), 1e-3, 'Name', 'DoublePendulum');

present(z)


%% create nonlinear greybox model

file_name = 'myfunc_DoublePendulum';
Order = [2 2 4];
Parameters = [1;1;1;1];
InitialStates = [pi/2;pi/2;0;0];

sys = idnlgrey(file_name,Order,Parameters,InitialStates,0, ...
    'Name','DoublePendulum');

% set(sys, 'InputName', {'no_input','no_input2'}, 'InputUnit', {'Nm','Nm'}, ...
%           'OutputName', {'theta1','theta2','omega1','omega2'}, ...
%           'OutputUnit', {'rad','rad', 'rad/s', 'rad/s'}, ...
%           'TimeUnit', 's');

set(sys, 'InputName', {'no_input','no_input2'}, 'InputUnit', {'Nm','Nm'}, ...
          'OutputName', {'theta1','theta2'}, ...
          'OutputUnit', {'rad','rad'}, ...
          'TimeUnit', 's');

% sys.Parameters(3).Fixed = true;
% sys.Parameters(4).Fixed = true;

setpar(sys,'Min',[0;0;0;0]);
setpar(sys,'Max',[10;10;10;10]);
% getpar(sys,'Min')
present(sys)

% predict(sys,z, [])
% compare(z, nlgr, [], compareOptions('InitialCondition', InitialStates));

%% 

nlgr = nlgreyest(sys, getexp(z, 1), nlgreyestOptions('Display', 'on'));

compare(z, nlgr, [], compareOptions('InitialCondition', InitialStates));
% err = pe(nlgr,z,1)
resid(z,nlgr)

%%

% opt = nlgreyestOptions;
% opt.Display = 'on';
% opt.SearchOptions.MaxIterations = 50;
% 
% load(fullfile(matlabroot,'toolbox','ident','iddemos','data','dcmotordata'));
% z = iddata(y,u,0.1,'Name','DC-motor');
% 
% file_name = 'dcmotor_m';
% Order = [2 1 2];
% Parameters = [1;0.28];
% InitialStates = [0;0];
% 
% init_sys = idnlgrey(file_name,Order,Parameters,InitialStates,0, ...
%     'Name','DC-motor');
% 
% sys = nlgreyest(z,init_sys,opt);





%%
function dx = double_pendulum_system(x,L1,L2,m1,m2,g)
% a system of differential equations defining a double pendulum
% from http://www.myphysicslab.com/dbl_pendulum.html
theta1 = x(1);
theta2 = x(2);
omega1 = x(3);
omega2 = x(4);
dtheta1 = omega1;
dtheta2 = omega2;
domega1 = (-g*(2*m1+m2)*sin(theta1)-m2*g*sin(theta1-2*theta2)-...
    2*sin(theta1-theta2)*m2*(omega2^2*L2+omega1^2*L1*cos(theta1-theta2)))...
    /(L1*(2*m1+m2-m2*cos(2*theta1-2*theta2)));
domega2 = (2*sin(theta1-theta2)*(omega1^2*L1*(m1+m2)+...
    g*(m1+m2)*cos(theta1)+omega2^2*L2*m2*cos(theta1-theta2)))...
    /(L2*(2*m1+m2-m2*cos(2*theta1-2*theta2)));
dx = [dtheta1;dtheta2;domega1;domega2];
end




