% test system 


clear all
close all
clc


%% build system 

% Transfer function
% first-order
T = 5;

numerator = 1;
denominator = [T, 1];

sys_1 = tf(numerator,denominator);
step(sys_1)


%% second-order

numerator = [1,5];
denominator = [.5,.8,5];
ts = 0.1;
sys_2_dis = tf(numerator,denominator,ts);
sys_2 = tf(numerator,denominator);
step(sys_2)

% nature frequency & damping ratio
% s_1,2 = -w0*zeta +- sqrt(zeta^2-1)
zeta = .4;
w0 = 1;
numerator = w0^2;
denominator = [1,2*zeta*w0,w0^2];
sys_2_std = tf(numerator,denominator);
% step(sys_2_std)

%% feedback system
K = tf([8,59],1); % PD 
PID_1 = pidtune(sys_2_std,'PID')
T = feedback(PID_1*sys_2_std,1);

%% Analysis
% time domain response
figure(1)
tFinal = 10;
h = stepplot(sys_2_std,T,tFinal);
% stepinfo(sys_1);
title('time domain step response')

