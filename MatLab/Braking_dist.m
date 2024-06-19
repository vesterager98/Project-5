clear all
close all
clc
syms s t vi
vmax = 36/3.6;
G = 1.981/(0.5938*s^2+s);      % Car TF (ignoring effects of motor)
tau = (vmax+vi)*5.1;                 % Maximum torque of the motor (Vmax*Gmotor(0))

v = vi - tau*ilaplace(G/s)           % Simulated step from eqv vi to -vmax
tf = solve(v == 0,t);                % Finding when car stops
x = int(v,t,0,tf)                    % Distance travelled

% Change to real part

X = matlabFunction(x);
Tf = matlabFunction(tf);
V = linspace(0,vmax,1000);

figure;
hold on;
grid on;

d_max = 1.7034; % m
vi_max = 10;     % m/s
syms vi;

f = @(vi) d_max/vi_max*vi

approx = f(V);
d_min = X(V);
t_min = Tf(V);
plot(V,approx,'b');
plot(V,d_min,'r');
legend("approx","d_min");



