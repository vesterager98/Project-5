clear all
close all
clc
s  = tf('s');

% Known Data:

K = 1.98;       % .
tau = 0.59;     % seconds
G = 1.98/s/(0.59*s+1); % Plant (Volts to m)
wc = 1.49; % rad/s
PM = 48.7; % º
%% Enhancing G_car
% Theoretically, plant is always stable by itself. Nevertheless, in reality
% it will become unstable before the 10 rad/s because of time delays. It is
% desired to study the error Z, thus the TF that it sees is not T but S,
% the sensibility of the system to external noise. S = 1 - T = 1/(1+L)
Kp = 5; 
T = feedback(G*Kp,1);
S = 1 - T;
bode(S)
%% Kp controller
% It can be observed that S --> 0 when w --> 0, so our system will not have
% stationary error with a proportional controller. Design requirements:

% - In stationary conditions (v = cte) error must be negligible
% - Overshoot must be smaller than car's nominal distance = 3cm !!! Real
% system will be allowed to increase this distance as velocity increases
% - Overshoot must be small enough for the following car to react
%   properly
% - Maximum distance between cars must never be greater than d_sensor

% System pole is too small for realizing this design without feedfoward:
% A new pole has been selected with the controller, aprox  8.5 times faster: 
D_pole = (s*0.59+1)/(0.07*s+1);
% This new trasnfer function can now be sped 8.5x more
Gnew = G*D_pole;
h1 = bodeplot(G,Gnew);
legend
grid on;

% Our main focus now is maintaining a high phase margin to reduce the
% overshoot into allowable levels (<10%). For doing so, PM must be 60º
% With a P controller, this phase margin can be obtained at w = 8.17 rad/s
% with Kp = 13.5 dB
Kp = 10^(13.5/20)
L = Kp*Gnew;
T = feedback(L,1);
figure
step(T)
S = 1-T;
figure
t = linspace(0,10,1000);
lsim(S,10*t,t);
%% Lag controller
close all
% The overshoot is maintained under the threshold peak value, but the
% system does not respond correctly against a speed disturbance: it lags up
% to 1 m behind!!! Solutions:
%   - Add an integrator the the controller: IMPOSIBLE, system becomes
%   unstable
%   - Increase Kp value
%   - Add PI control: IMPSIBLE, system becomes unstable
%
% Thus, the Kp of the system must be increased. Assuming v_max = 10m/s,
% and a maximum allowance of distance variation of 15cm:
% |S| = 1/(K*Kp)
Kp_needed = 10/0.15/1.98;
% Kp = 33.67. PM would be 26º!!! P control is not enough: lead and lag
% controls can help solve this issue. Lead control will not be considered
% in case the real system has too high of a PM drop, the lead can then be
% added into the design.
% With a lag controller the previous Kp = 4.73 can be kept. The payoff is
% reducing the speed at which the disturbance is eliminated:
% disturbance ts < 5s -> sigma > 1.6
% Also,the lag controller must not affect the PM of the system: can not
% operate faster than 1 rad/s -> design will NOT respect the 5s rule.
% Scaling of lag controller must be: Kp_needed/Kp = 7.12
% Choose fastest lag controller: zero at 0.8 rad/s:

lag = (s+0.8)/(s+0.8/7.12)
L = lag*Gnew*Kp;
bode(L);
T = feedback(L,1);
step(T);
grid on
S = 1-T;
figure
t = linspace(0,10,1000);
lsim(S,10*t,t);
% It can be observed that both the Mp and the d contraints are fulfilled.
% Still, the disturbance rejection is too slow to take place (5.5), causing
% the value to peak to 1.2m before it can take place, which makes sense as
% Kp = 4.7 and not 33 until the lag controller takes effect.
% Therefore, a normal controller design can not be used for this purpose
% Feedfoward is needed for this task, to reduce the inicial effect of the
% step.

% Thus, this controller ends up as:

D = (0.59*s+1)/(0.07*s+1)*(s+0.8)/(s+0.1124)*4.73


%% Foward control
close all
% As defined in the previous part, a new kind of disturbance rejection is
% needed, thus the use of a feedfoward. For using a more real approach, it
% will be assumed that the leading car posses the same dynamics of the
% following car, this is:
G_L = 1/(0.59*s+1); % V_L/V_L_ref

% Moreover, the feedfoward loop changes the TF w.r.t. the disturbance
% rejection from
% W = G_L*S (velocity)
% to
% W = S*(1-H2*M*G*s)*G_L (velocity)
% Where H2 is the speed sensor (assumed 1) and M the feedfoward controller
% Thus, it has to be studied how to reduce the value of W from 10 to
% 0.7 so that, with the previously defined controller, S < 0.15 always.
% Ideally, M = 1/(G*H) so disturbances have no effect on the system. Sadly,
% this approach is not possible as G has more poles than zeros, and thus M
% would be impossible to build. Still, a difficult design could be carried
% out, assigning poles to M that do not affect the system.
% A more realistic approach is to first eliminate the error in an infinite
% amount of time, this is: |H*M*G| = 1
% Thus, |M| = 1/|G|
M_kp = 1/1.98;
% This feedfoward would have a settling time of ts = 5.01 s, only slightly
% better than using the lag control.
% To improve this, the dominant pole of MG can be changed:
M = M_kp*(0.59*s+1)/(0.07*s+1);


T = feedback(L,1);
S = 1-T;
t = linspace(0,10,1000);
W = G_L*(1-M*G*s);

step(W*10/s);
% It can be seen that the disturbance has now been reduced a type: now a
% ramp generates a step disturbance to be treated. Thus, now a lag control
% is not needed as the plant is already a type 1 error rejection system.
% Therefore, the maximum disturbance will generate the graph:
step(10/s*W*S);
% That ensures the following car will not get further than 20 cms from the
% leading car

%% D_ref w.r.t velocity
% One last issue arises in this control: 
% The leading car separates 15 cm from the follower car (a total of 20cm),
% before coming back tot eh 5cm. Therefore, when the car passes from 10m/s
% to 0 (brakes), there must be at least 15 cm between the cars so that they
% do not collide: Ideal distance between cars changes dinamically
% between 5cm and 20cm. This effect can be attained in 2 ways:
%  - Using the built in error of the system to ramp inputs(can not be done, as the overall system is of type 2 against disturbances)
%  - Changing the distance reference dinamically w.r.t v_F

% Thus, the distance reference must change w.r.t v_F. still, this effect
% must be employed carefully as increasing the distance reference
% translates into a virtual decrease of the Kp, and thus enlarging once
% again the maximum disturbance in distance between cars.
% In steady state, 10 m/s must add 15cm -> |d_v_ref| = 0.15/10 = 0.015
% Thankfully, this addition does not cause the maximum distance between
% cars to overshoot over 20cm,when simulated with simulink

% This means that a succesful controller has been designed, using:
% - P-pole enhancement controller: D = 4.7*(0.59*s+1)/(0.07*s+1)
% - Feedfoward of v_L: M = 0.505*(0.59*s+1)/(0.59*s+1)
% - Reference variation wrt v_F: N = 0.015

% Improvements: 
%  - The system's PM can be enhanced with a lead controller
%    without repercussions, as high noise is not a problem in this system
%  - The motor is not driven into saturation -> the controller can be more
%    agresive.
