clear all

% Lorenz's parameters (chaotic)
sigma = 10;
beta = 8/3;
rho = 28;

% Initial condition 1 - Large cube of data
y0=[0 0 0];
xvec = -20:2:20;
yvec = -20:2:20;
zvec = -20:2:20;

% % Initial condition 2 - small cube around initial condition from last class
% y0=[-8; 8; 27];
% xvec = -1:.1:1;
% yvec = -1:.1:1;
% zvec = -1:.1:1;

% % Initial condition 3 - even smaller cube around initial condition
% y0=[-8; 8; 27];
% xvec = -.1:.01:.1;
% yvec = -.1:.01:.1;
% zvec = -.1:.01:.1;


[x0,y0,z0] = meshgrid(xvec+y0(1),yvec+y0(2),zvec+y0(3));
yIC(1,:,:,:) = x0;
yIC(2,:,:,:) = y0;
yIC(3,:,:,:) = z0;

plot3(yIC(1,:),yIC(2,:),yIC(3,:),'r.','LineWidth',2,'MarkerSize',4)
axis([-40 40 -40 40 -40 40])
view(20,40);
drawnow

%% Compute trajectory 
dt =0.01;
duration = 10;
tspan=[0,duration]; 
L = duration/dt; 
yin = yIC(1:3,:); 



for step=1:L
    time = step*dt; 
    yout = RK_4(@(t,y)lorenz_vector(t,y,sigma,beta,rho),dt,time,yin);
    yin = yout;
    plot3(yin(1,:),yin(2,:),yin(3,:),'r.','LineWidth',2,'MarkerSize',2)
    view(20 + step*360/L,40);
    axis([-40 40 -40 40 -10 70])
    drawnow
end