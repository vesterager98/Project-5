%% This file can compute the values of a TF with a leading real pole that presents a ramp-like output
% (step input for a TF with pole at s=0)

clear all 
close all
clc

% Ramp values (remember to use SI units):
m = 380; % Slope of steady-state ramp
n = -24.375;   % Y-axis intercept 
V_in = 5; % Applied input size

% Calculations ------------------------------------------------------
syms s A B V
disp("The ramp response should follow the formula:");
G = 1/(s^2*(A*s+B));
alpha = ilaplace(G*V);

V = V_in;
B = V/m;
A = (-n)*B^2/V;

disp("And the TF is:");
s  = tf('s');
G = (1/B)/(A/B*s+1)/s



%Plotting

tau = A/B;
t = linspace(0,5*tau,1000);
x = m*t;
[y,t] = step(G*V,t);
figure;
hold on; grid on;
xlim([0 2.5])
ylim([-0.4+min(y) 0.6+max(y)])

plot(t,y,'b');
plot(t,m*t+n,'r');
