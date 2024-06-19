close all;
clear all;
clc
%Plant
s = tf('s');
VtoS = 76/s/(0.065*s+1);
StoW = 0.72;
%Sensor at VtoS
G = VtoS*StoW;
bode(G)
Dpole = (0.065*s+1)/(0.007*s+1) 
Gnew = Dpole*G;
bode(G,Gnew)
legend
grid on
hold on
%% P controller
% We want wc = 100rad/s

close all
[mag phase freq] = bode(Gnew,100);
Kp = 1/mag
D = Kp;
L = Gnew*Kp;
bode(L)

% This controller suffices for the purpose


