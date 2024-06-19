% Data filtering and plotting

meas = csvread('Data1.txt');

% Angle signal: 
% 519 is upright position 
% 290 corresponds to ~90 deg or pi/2
ang = 0.31*(meas(:,1) - 368);
% Reference signal
ref = 0.31*(meas(:,2) - 368);
% The (reference)current = -13.75+0.0269*duty [A]
duty = 0.0269*meas(:,3) - 13.75;

% Timing
Ts = 0.005;
t =0:Ts:Ts*(length(duty)-1);

% If the position signal is noisy the following filtering is useful:
% NB: filtering position forwards and backwards to ensure no delay
s = tf('s'); tau = 0.05; G = 1/(tau*s+1);
angf1 = lsim(G, ang'-mean(ang), t); angf2 = flipud(angf1);
angf3 = lsim(G, angf2, t) + mean(ang); angf = flipud(angf3);

%model = tf([112.19],[1, 2.83, 112.19]);
model1 = tf([18.29],[1, 1.94, 18.29]);
%model2 = tf([7.44],[1, 2.76, 7.44]);
Stepopts = stepDataOptions('InputOffset',-46.5,'StepAmplitude',2*46.5);
Modelstep1 = step(model1,t,Stepopts);
%Modelstep2 = step(model2,t,Stepopts);

figure(1);
plot(t, ref, t, ang);
xlabel('Time [s]');
ylabel('Angle [deg]');
legend('Angle Reference', 'Angle measurement');
xlim([0, t(end)])
grid on;

figure(2);
plot(t, duty);
xlabel('Time [s]');
ylabel('Current [A]');
xlim([0, t(end)])
grid on;

figure(3);
plot(t,ref,t,angf,t+3.98,Modelstep1);
xlabel('Time [s]');
ylabel('Angle [deg]');
legend('Angle Reference', 'Angle filtered', 'Model');
xlim([0, t(end)])
grid on;

figure(4);
plot(t,ref,t,ang,t+3.98,Modelstep1);
xlabel('Time [s]');
ylabel('Angle [deg]');
legend('Angle Reference', 'Angle measurement', 'Model');
xlim([0, t(end)])
grid on;

