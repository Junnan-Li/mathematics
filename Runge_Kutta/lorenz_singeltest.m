clear all

% Lorenz's parameters (chaotic)
sigma = 10;
beta = 8/3;
rho = 28;

% Initial condition
y0=[-8; 8; 27];
y1=[-8; 8.2; 26.9];

% Compute trajectory 
dt =0.01;
tspan=[0:dt:4]; 

Y(:,1)=y0;
Y1(:,1)= y1;
yin = y0;
yin1 = y1;
for i=1:tspan(400)/dt
    time = i*dt;
    yout = RK_4(@(t,y)lorenz(t,y,sigma,beta,rho),dt,time,yin);
    yout1 = RK_4(@(t,y)lorenz(t,y,sigma,beta,rho),dt,time,yin1);
    Y = [Y yout];
    Y1 = [Y1 yout1];
    yin = yout;
    yin1 = yout1;
end
% plot3(Y(1,:),Y(2,:),Y(3,:),'b')
% hold on
% plot3(Y1(1,:),Y1(2,:),Y1(3,:),'r')
[t,y] = ode45(@(t,y)lorenz(t,y,sigma,beta,rho),tspan,y0);
plot3(y(:,1),y(:,2),y(:,3),'b')
hold on
[t1,y1] = ode45(@(t,y)lorenz(t,y,sigma,beta,rho),tspan,y1);
plot3(y1(:,1),y1(:,2),y1(:,3),'r')