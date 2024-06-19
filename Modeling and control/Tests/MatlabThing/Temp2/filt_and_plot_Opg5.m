% Data filtering and plotting

meas0 = csvread('data0.txt');
meas1 = csvread('data1.txt');
meas2 = csvread('data2.txt');
meas3 = csvread('data3.txt');

% Angle signal: 
% 519 is upright position 
% 290 corresponds to ~90 deg or pi/2
ang1 = 0.31*(meas1(:,1) - 368);
% Reference signal
ref1 = 0.31*(meas1(:,2) - 368);

ang2 = 0.31*(meas2(:,1) - 368);
% Reference signal
ref2 = 0.31*(meas2(:,2) - 368);

ang3 = 0.31*(meas3(:,1) - 368);
% Reference signal
ref3 = 0.31*(meas3(:,2) - 368);

ang0 = 0.31*(meas0(:,1) - 368);
% Reference signal
ref0 = 0.31*(meas0(:,2) - 368);
% The (reference)current = -13.75+0.0269*duty [A]
duty1 = 0.0269*meas1(:,3) - 13.75;
duty2 = 0.0269*meas2(:,3) - 13.75;
duty3 = 0.0269*meas3(:,3) - 13.75;
duty0 = 0.0269*meas0(:,3) - 13.75;

% Timing
Ts = 0.005;
t1 =0:Ts:Ts*(length(duty1)-1);
t2 =0:Ts:Ts*(length(duty2)-1);
t3 =0:Ts:Ts*(length(duty3)-1);
t0 =0:Ts:Ts*(length(duty0)-1);

% If the position signal is noisy the following filtering is useful:
% NB: filtering position forwards and backwards to ensure no delay

s = tf('s'); tau = 0.05; G = 1/(tau*s+1);
angf11 = lsim(G, ang1'-mean(ang1), t1); angf21 = flipud(angf11);
angf31 = lsim(G, angf21, t1) + mean(ang1); angf1 = flipud(angf31);
angf12 = lsim(G, ang2'-mean(ang2), t2); angf22 = flipud(angf12);
angf32 = lsim(G, angf22, t2) + mean(ang2); angf2 = flipud(angf32);
angf13 = lsim(G, ang3'-mean(ang3), t3); angf23 = flipud(angf13);
angf33 = lsim(G, angf23, t3) + mean(ang3); angf3 = flipud(angf33);
angf10 = lsim(G, ang0'-mean(ang0), t0); angf20 = flipud(angf10);
angf30 = lsim(G, angf20, t0) + mean(ang0); angf0 = flipud(angf30);


%model = tf([112.19],[1, 2.83, 112.19]);
%model = tf([13.77],[1, 1.29, 13.77]);
%Stepopts = stepDataOptions('InputOffset',-46.5,'StepAmplitude',2*46.5);
%Modelstep = step(model,t,Stepopts);

figure(1);
plot(t1, ref1, t0-2.13, ang0, t1, ang1, t2+0.115, ang2, t3-1.635, ang3);
xlabel('Time [s]');
ylabel('Angle [deg]');
legend('Angle Reference', 'Kv = 0.25', 'Kv = 1', 'Kv = 3');
xlim([0, t(end)])
grid on;


figure(3);
plot(t1, ref1, t0-2.13, angf0, t1, angf1, t2+0.115, angf2, t3-1.635, angf3);
xlabel('Time [s]');
ylabel('Angle [deg]');
legend('Angle Reference', 'Kv = 0', 'Kv = 0.25', 'Kv = 1', 'Kv = 3');
xlim([0, t(end)])
grid on;
