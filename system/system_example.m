% test system 


clear all
close all
clc


%% build system 

% Transfer function
% first-order
T = 1;

numerator = 1;
denominator = [T, 1];

sys_1 = tf(numerator,denominator);

% second-order

numerator = [1,4];
denominator = [2,3,4];
ts = 0.1;
sys_2_dis = tf(numerator,denominator,ts);
sys_2 = tf(numerator,denominator);

% nature frequency & damping ratio
% s_1,2 = -w0*zeta +- sqrt(zeta^2-1)
zeta = .4;
w0 = 1;
numerator = w0^2;
denominator = [1,2*zeta*w0,w0^2];
sys_2_std = tf(numerator,denominator);

% feedback system
K = tf([8,59],1); % PD 
PID_1 = pidtune(sys_2_std,'PD')
T = feedback(PID_1*sys_2_std,1);

%% Analysis
% time domain response
figure(1)
tFinal = 50;
h = stepplot(sys_2_std,T,tFinal);
% stepinfo(sys_1);
title('time domain step response')

% impulse(sys_1);

% figure(2)
% t = 0:0.04:8;  % 201 points 
% u = max(0,min(t-1,1));
% lsim(sys_2_std,u,t) 
% grid on
% title('time domain input response')

% figure(3)
% pzplot(sys_2_std)
% grid on
% axis equal
% title('pole zero plot')
% p = pole(sys_2_std)
