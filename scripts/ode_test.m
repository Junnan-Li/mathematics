
% dy/dt = -y



y0 = -3;
dt = 1e-3;
t0 = 0;

fun = @(t,y) -3*y + sin(t);

T = [];
T(1) = t0;

Y_rk = [];
Y_rk(1) = y0;

Y_eu = [];
Y_eu(1) = y0;

Y_mid = [];
Y_mid(1) = y0;

Y_heun = [];
Y_heun(1) = y0;



for i = 1:10000
    
    t = T(i);
    
    % runge kutta 4 solver
    yn_rk = Y_rk(i);
    y_n1_rk = rungekutta4(fun, dt,t,yn_rk);
    Y_rk(i+1) = y_n1_rk;
    
    
    % euler solver
    yn_eu = Y_eu(i);
    y_n1_eu = euler(fun, dt,t,yn_eu);
    Y_eu(i+1) = y_n1_eu;
    
    % midpoint solver
    yn_mid = Y_mid(i);
    y_n1_mid = midpoint(fun, dt,t,yn_mid);
    Y_mid(i+1) = y_n1_mid;
    
    % heun solver
    yn_heun = Y_heun(i);
    y_n1_heun = heun(fun, dt,t,yn_heun);
    Y_heun(i+1) = y_n1_heun;
    
    T(i+1) = t + dt;
end

% ode45 solver
tspan = [0 10];
[t_ode45,y_ode45] = ode45(@(t,y) -3*y, tspan, y0);


figure(1)
plot(T(:),Y_rk(:),'r')
hold on
plot(T(:),Y_eu(:),'g')
hold on
plot(T(:),Y_mid(:),'c')
hold on
plot(T(:),Y_heun(:),'k')
hold on
% plot(t_ode45(:),y_ode45(:),'b')
legend('rk4','euler','midpoint','heun')







