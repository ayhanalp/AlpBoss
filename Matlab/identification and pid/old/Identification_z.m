%%% Parameter estimation %%%
% zie ook 'test.m' !
clear all
close all

%% Load data from .mat-file

load vel_identification_z_short


%% Extract some signals from the data

set(0, 'DefaultLineLineWidth', 1);


% Cutoff useful data
% first_index = find(input~=0, 1);
input = input(1:end-100)';
output_x = output_x(1:end-100)';
output_y = output_y(1:end-100)';
output_z = output_z(1:end-100)' - output_z(1);

% Sample time
dt = 0.02;
time = (0:dt:(length(input)-1)*dt)';

% Differentiation of x-, y-, z-position
% dt = sample time (gradient assumes timestep 1)
velocity_x = gradient(output_x)/dt;
velocity_y = gradient(output_y)/dt;
velocity_z = gradient(output_z)/dt;

% Do some plotting of the measurement data.
figure('Name','Measurement Data')
subplot(321), plot(time, output_x), title('Position x'), xlabel('time [s]'), ylabel('position [m]')
subplot(322), plot(time, velocity_x), title('Velocity x'), xlabel('time [s]'), ylabel('speed [m/s]')

subplot(323), plot(time, output_y), title('Position y'), xlabel('time [s]'), ylabel('position [m]')
subplot(324), plot(time, velocity_y), title('Velocity y'), xlabel('time [s]'), ylabel('speed [m/s]')

subplot(325), plot(time, output_z, time, input), title('Position z'), xlabel('time [s]'), ylabel('position [m]'), legend('output','input')
subplot(326), plot(time, velocity_z), title('Velocity z'), xlabel('time [s]'), ylabel('speed [m/s]')


%% Frequency domain - Empirical Transfer Function
Ts = dt;
fs = 1/dt;
N = length(velocity_x);
t = [0:N-1]'*Ts;
N = numel(input);
f = [0:N-1]'*(fs/N);

input_f = fft(input);
output_z_f = fft(output_x);
velocity_x_f = fft(velocity_x);

FRF = output_z_f./input_f;

figure('Name', 'Empirical transfer function freq response'),subplot(2,1,1),semilogx(f, 20*log10(abs(FRF)), 'LineWidth', 1)
axis tight
grid on
xlabel('f [Hz]')
xlim([f(1) f(end)])
ylabel('|FRF| [m]')
subplot(2,1,2),semilogx(f, 180/pi*unwrap(angle(FRF)), 'LineWidth', 1)
grid on
axis tight
xlabel('f  [Hz]')
ylabel('\phi(FRF) [^\circ]')
xlim([f(1) f(end)])


%% Choose a cutoff frequency for Butterworth filtering
f0 = 0.2; % crossover frequency
fc = 5*f0; % cutoff frequency
fcn = fc/(fs/2); % normalized cutoff frequency



%% ---------------------
%   SECOND ORDER FITTING 
%  ---------------------

%% Filtering of the in- and output data using Butterworth filter
[B, A] = butter(3, fcn); % order must be higher than order of system, 
                            % adjust cut-off frquency to be higher than 
                            % highest eigenfrequency of the system
                             
% input filtering                           
input_filt = filter(B, A, input);
% output filtering
output_z_filt = filter(B, A, output_z);

figure('Name','3d order Butt. filtered Input & Measurement')
subplot(211),plot(t,input,t,input_filt),title('Input filtered')
legend('raw input', 'filtered input')
subplot(212),plot(time, output_z, time, output_z_filt),title('pos_{z,filt}')
legend('raw measurement', 'filtered measurement')


% Least squares solution for approximation of the parameters in the system
%% Without filtering
% % --- PROPER [order(N) = order(D)]
% %
% %           Y(z)    b2*z^2 + b1*z + b0
% %   G(z) = ----- = --------------------
% %           U(z)     z^2 + a1*z + a0
% %
% % y[k] = -a1*y[k-1]-a0*y[k-2]+b2*u[k]+b1*u[k-1]+b0*u[k-2]
% %
% % theta = [a1 a0 b2 b1 b0]
% 
% y1 = output_z(3:end);
% Phi1 = [-output_z(2:end-1), -output_z(1:end-2), input(3:end), input(2:end-1), input(1:end-2)];
% theta1 = Phi1\y1;
% 
% B1 = [theta1(3), theta1(4), theta1(5)];
% A1 = [1, theta1(1) theta1(2)];
% 
% sys_d1 = tf(B1, A1, Ts);
% 
% FRF1 = squeeze(freqresp(sys_d1,2*pi*f));
% 
% figure('Name','2nd order, no filter, proper - Freq. resp.'), subplot(211)
% semilogx(f, 20*log10(abs(FRF1)))
% grid on
% xlim([f(1) f(end)])
% xlabel('f  [Hz]')
% ylabel('|FRF1|  [m]')
% subplot(212),semilogx(f, 180/pi*unwrap(angle(FRF1)))
% grid on
% xlim([f(1) f(end)])
% xlabel('f  [Hz]')
% ylabel('\phi(FRF1)  [^\circ]')
% 
% x1 = lsim(sys_d1,input,t);
% 
% figure('Name','2nd order, no filter, proper - Simulation')
% subplot(211)
% hold on
% plot(t, output_z,'g')
% plot(t, x1)
% title('2nd order, no filter, proper - Simulation VS Measurement')
% legend('pos_{z,meas}','pos_{z,sim}')
% xlabel('Time [s]')
% ylabel('Displacement [m]')
% axis tight
% subplot(212)
% plot(t,output_z - x1)
% title('Difference between simulation and measurement')
% legend('pos_{z,meas}-pos_{z,sim}')
% xlabel('Time [s]')
% ylabel('Displacement [m]')
% axis tight
% 
% figure('Name','2nd order, no filter, proper - Pole Zero Map'),pzmap(sys_d1)


%% With Butterworth filtering of in and output
% % --- PROPER [order(N) = order(D)]
% %
% %           Y(z)    b2*z^2 + b1*z + b0
% %   G(z) = ----- = --------------------
% %           U(z)     z^2 + a1*z + a0
% %
% % y[k] = -a1*y[k-1]-a0*y[k-2]+b2*u[k]+b1*u[k-1]+b0*u[k-2]
% %
% % theta = [a1 a0 b2 b1 b0]
% 
% y2 = output_z_filt(3:end);
% Phi2 = [-output_z_filt(2:end-1), -output_z_filt(1:end-2), input_filt(3:end), input_filt(2:end-1), input_filt(1:end-2)];
% theta2 = Phi2\y2;
% 
% B2 = [theta2(3),theta2(4),theta2(5)];
% A2 = [1, theta2(1) theta2(2)];
% 
% sys_d2 = tf(B2, A2, Ts);
% 
% FRF2 = squeeze(freqresp(sys_d2,2*pi*f));
% 
% figure('Name','2nd order, filtered, proper - Freq. resp.'),subplot(2,1,1),semilogx(f, 20*log10(abs(FRF2)))
% grid on 
% xlim([f(1) f(end)])
% xlabel('f  [Hz]')
% ylabel('|FRF2|  [m]')
% axis tight
% subplot(2,1,2)
% semilogx(f, 180/pi*unwrap(angle(FRF2)))
% grid on
% xlim([f(1) f(end)])
% xlabel('f  [Hz]')
% ylabel('\phi(FRF2)  [^\circ]')
% 
% x2 = lsim(sys_d2,input,t);
% 
% figure('Name','2nd order, filtered, proper - Simulation')
% subplot(211)
% hold on
% plot(t, output_z,'g')
% plot(t, output_z_filt)
% plot(t,x2)
% legend('pos_{z,meas}', 'pos_{z,filt}', 'pos_{z,sim}')
% title('2nd order, filtered, proper - Simulation VS Measurement')
% xlabel('Time [s]')
% axis tight
% ylabel('output [m/s]')
% subplot(212)
% plot(t,output_z - x2)
% title('Difference between simulation and measurement')
% legend('pos_{z,meas}-pos_{z,sim}')
% xlabel('Time [s]')
% ylabel('output [m/s]')
% axis tight
% 
% figure('Name','2nd order, filtered, proper - Pole Zero Map'),pzmap(sys_d2)

%% With Butterworth filtering of in and output
% --- STRICTLY PROPER [order(N) = order(D)]
%
%           Y(z)        b1*z + b0
%   G(z) = ----- = --------------------
%           U(z)     z^2 + a1*z + a0
%
% y[k] = -a1*y[k-1]-a0*y[k-2]+b2*u[k]+b1*u[k-1]+b0*u[k-2]
%
% theta = [a1 a0 b1 b0]

y0 = output_z_filt(3:end);
% Phi0 = [-output_z_filt(2:end-1), -output_z_filt(1:end-2), input_filt(2:end-1), input_filt(1:end-2)];
Phi0 = [-output_z_filt(2:end-1), -output_z_filt(1:end-2), input_filt(1:end-2)];

theta0 = Phi0\y0;

% B0 = [theta0(3),theta0(4)];
B0 = [theta0(3)];

A0 = [1, theta0(1) theta0(2)];

sys_d0 = tf(B0, A0, Ts);

FRF0 = squeeze(freqresp(sys_d0,2*pi*f));

figure('Name','2nd order, filtered, strictly proper - Freq. resp.'),subplot(2,1,1),semilogx(f, 20*log10(abs(FRF0)))
grid on 
xlim([f(1) f(end)])
xlabel('f  [Hz]')
ylabel('|FRF0|  [m]')
axis tight
subplot(2,1,2)
semilogx(f, 180/pi*unwrap(angle(FRF0)))
grid on
xlim([f(1) f(end)])
xlabel('f  [Hz]')
ylabel('\phi(FRF0)  [^\circ]')

x0 = lsim(sys_d0,input,t);

figure('Name','2nd order, filtered, strictly proper - Simulation')
subplot(211)
hold on
plot(t, output_z,'g')
plot(t, output_z_filt)
plot(t,x0)
legend('pos_{z,meas}', 'pos_{z,filt}', 'pos_{z,sim}')
title('2nd order, filtered, proper - Simulation VS Measurement')
xlabel('Time [s]')
axis tight
ylabel('output [m/s]')
subplot(212)
plot(t,output_z - x0)
title('Difference between simulation and measurement')
legend('pos_{z,meas}-pos_{z,sim}')
xlabel('Time [s]')
ylabel('output [m/s]')
axis tight

figure('Name','2nd order, filtered, proper - Pole Zero Map'),pzmap(sys_d0)



%% ---------------------
% %   THIRD ORDER FITTING 
% %  ---------------------
% 
% %% Filtering of the in- and output data using Butterworth filter
% [B, A] = butter(4, fcn); % order must be higher than order of system, 
%                             % adjust cut-off frquency to be higher than 
%                             % highest eigenfrequency of the system
%                              
% % input filtering                           
% input_filt = filter(B, A, input);
% % output filtering
% output_z_filt = filter(B, A, output_z);
% 
% figure('Name','4th order Butt. filtered Input & Measurement')
% subplot(211),plot(t,input,t,input_filt),title('Input filtered')
% legend('raw input', 'filtered input')
% subplot(212),plot(time, output_z, time, output_z_filt),title('pos_{z,filt}')
% legend('raw measurement', 'filtered measurement')
% 
% 
% % Least squares solution for approximation of the parameters in the system
% %% Without filtering
% % --- PROPER [order(N) = order(D)]
% %
% %           Y(z)    b3*z^3 + b2*z^2 + b1*z + b0
% %   G(z) = ----- = -----------------------------
% %           U(z)     z^3 + a2*z^2 + a1*z + a0
% %
% % y[k] =
% % -a2*y[k-1]-a1*y[k-2]-a0*y[k-3]+b3*u[k]+b2*u[k-1]+b1*u[k-2]+b0*u[k-3]
% %
% % theta = [a2 a1 a0 b3 b2 b1 b0]
% 
% y3 = output_z(4:end);
% Phi3 = [-output_z(3:end-1), -output_z(2:end-2), -output_z(1:end-3), input(4:end), input(3:end-1), input(2:end-2), input(1:end-3)];
% theta3 = Phi3\y3;
% 
% B3 = [theta3(4), theta3(5), theta3(6), theta3(7)];
% A3 = [1, theta3(1) theta3(2) theta3(3)];
% 
% sys_d3 = tf(B3, A3, Ts);
% 
% FRF3 = squeeze(freqresp(sys_d3,2*pi*f));
% 
% figure('Name','3d order, no filter, proper -  Freq. Resp.'), subplot(211)
% semilogx(f, 20*log10(abs(FRF3)))
% grid on
% xlim([f(1) f(end)])
% xlabel('f  [Hz]')
% ylabel('|FRF3|  [m]')
% subplot(212),semilogx(f, 180/pi*unwrap(angle(FRF3)))
% grid on
% xlim([f(1) f(end)])
% xlabel('f  [Hz]')
% ylabel('\phi(FRF3)  [^\circ]')
% 
% x3 = lsim(sys_d3,input,t);
% 
% figure('Name','3d order, no filter, proper - Simulation')
% subplot(211)
% hold on
% plot(t, output_z,'g')
% plot(t, x3)
% title('3d order, no filter, proper - Simulation VS Measurement')
% legend('pos_{z,meas}','pos_{z,sim}')
% xlabel('Time [s]')
% ylabel('Displacement [m]')
% axis tight
% subplot(212)
% plot(t,output_z - x3)
% title('Difference between simulation and measurement')
% legend('pos_{z,meas}-pos_{z,sim}')
% xlabel('Time [s]')
% ylabel('Displacement [m]')
% axis tight
% 
% figure('Name','3d order, no filter, proper - Pole Zero Map'),pzmap(sys_d3)
% 
% 
% %% With Butterworth filtering of in- and output.
% % --- PROPER [order(N) = order(D)]
% %
% %           Y(z)    b3*z^3 + b2*z^2 + b1*z + b0
% %   G(z) = ----- = -----------------------------
% %           U(z)     z^3 + a2*z^2 + a1*z + a0
% %
% % y[k] =
% % -a2*y[k-1]-a1*y[k-2]-a0*y[k-3]+b3*u[k]+b2*u[k-1]+b1*u[k-2]+b0*u[k-3]
% %
% % theta = [a2 a1 a0 b3 b2 b1 b0]
% 
% y4 = output_z_filt(4:end);
% Phi4 = [-output_z_filt(3:end-1), -output_z_filt(2:end-2), -output_z_filt(1:end-3), input_filt(4:end), input_filt(3:end-1), input_filt(2:end-2), input_filt(1:end-3)];
% theta4 = Phi4\y4;
% 
% B4 = [theta4(4), theta4(5), theta4(6), theta4(7)];
% A4 = [1, theta4(1) theta4(2) theta4(3)];
% 
% sys_d4 = tf(B4, A4, Ts);
% 
% FRF4 = squeeze(freqresp(sys_d4,2*pi*f));
% 
% figure('Name','3d order, filtered, proper - Freq. Resp.'), subplot(211)
% semilogx(f, 20*log10(abs(FRF4)))
% grid on
% xlim([f(1) f(end)])
% xlabel('f  [Hz]')
% ylabel('|FRF4|  [m]')
% subplot(212),semilogx(f, 180/pi*unwrap(angle(FRF4)))
% grid on
% xlim([f(1) f(end)])
% xlabel('f  [Hz]')
% ylabel('\phi(FRF4)  [^\circ]')
% 
% x4 = lsim(sys_d4,input,t);
% 
% figure('Name','3d order, filtered, proper - Simulation')
% subplot(211)
% hold on
% plot(t, output_z_filt,'g')
% plot(t, x4)
% title('3d order, filtered, proper - Simulation VS Measurement')
% legend('pos_{z,meas}','pos_{z,sim}')
% xlabel('Time [s]')
% ylabel('Displacement [m]')
% axis tight
% subplot(212)
% plot(t,output_z_filt - x4)
% title('Difference between simulation and measurement')
% legend('pos_{z,meas}-pos_{z,sim}')
% xlabel('Time [s]')
% ylabel('Displacement [m]')
% axis tight
% 
% figure('Name','3d order, filtered, proper - Pole Zero Map'),pzmap(sys_d4)
% 
% %% Without filtering
% % --- STRICTLY PROPER [order(N) < order(D)]
% %
% %           Y(z)       b2*z^2 + b1*z + b0
% %   G(z) = ----- = --------------------------
% %           U(z)    z^3 + a2*z^2 + a1*z + a0
% %
% % y[k] = -a2*y[k-1]-a1*y[k-2]-a0*y[k-3]+b2*u[k-1]+b1*u[k-2]+b0*u[k-3]
% %
% % theta = [a2 a1 a0 b2 b1 b0]
% 
% y5 = output_z(4:end);
% Phi5 = [-output_z(3:end-1), -output_z(2:end-2), -output_z(1:end-3), input(4:end), input(3:end-1), input(2:end-2), input(1:end-3)];
% theta5 = Phi5\y5;
% 
% B5 = [theta5(4), theta5(5), theta5(6)];
% A5 = [1, theta5(1) theta5(2) theta5(3)];
% 
% sys_d5 = tf(B5, A5, Ts);
% 
% FRF5 = squeeze(freqresp(sys_d5,2*pi*f));
% 
% figure('Name','3d order, no filter, strictly proper - Freq. Resp.'), subplot(211)
% semilogx(f, 20*log10(abs(FRF5)))
% grid on
% xlim([f(1) f(end)])
% xlabel('f  [Hz]')
% ylabel('|FRF5|  [m]')
% subplot(212),semilogx(f, 180/pi*unwrap(angle(FRF5)))
% grid on
% xlim([f(1) f(end)])
% xlabel('f  [Hz]')
% ylabel('\phi(FRF5)  [^\circ]')
% 
% x5 = lsim(sys_d5,input,t);
% 
% figure('Name','3d order, no filter, strictly proper - Simulation')
% subplot(211)
% hold on
% plot(t, output_z,'g')
% plot(t, x5)
% title('3d order, no filter, strictly proper - Simulation VS Measurement')
% legend('pos_{z,meas}','pos_{z,sim}')
% xlabel('Time [s]')
% ylabel('Displacement [m]')
% axis tight
% subplot(212)
% plot(t,output_z - x5)
% title('Difference between simulation and measurement')
% legend('pos_{z,meas}-pos_{z,sim}')
% xlabel('Time [s]')
% ylabel('Displacement [m]')
% axis tight
% 
% figure('Name','3d order, no filter, strictly proper - Pole Zero Map'),pzmap(sys_d5)
% 
% 
% %% With Butterworth filtering of in- and output.
% % --- STRICTLY PROPER [order(N) < order(D)]
% %
% %           Y(z)       b2*z^2 + b1*z + b0
% %   G(z) = ----- = --------------------------
% %           U(z)    z^3 + a2*z^2 + a1*z + a0
% %
% % y[k] = -a2*y[k-1]-a1*y[k-2]-a0*y[k-3]]+b2*u[k-1]+b1*u[k-2]+b0*u[k-3]
% %
% % theta = [a2 a1 a0 b2 b1 b0]
% 
% y6 = output_z_filt(4:end);
% Phi6 = [-output_z_filt(3:end-1), -output_z_filt(2:end-2), -output_z_filt(1:end-3), input_filt(3:end-1), input_filt(2:end-2), input_filt(1:end-3)];
% theta6 = Phi6\y6;
% 
% B6 = [theta6(4), theta6(5), theta6(6)];
% A6 = [1, theta6(1) theta6(2) theta6(3)];
% 
% sys_d6 = tf(B6, A6, Ts);
% 
% FRF6 = squeeze(freqresp(sys_d6,2*pi*f));
% 
% figure('Name','3d order, filtered, strictly proper - Freq. Resp.'), subplot(211)
% semilogx(f, 20*log10(abs(FRF6)))
% grid on
% xlim([f(1) f(end)])
% xlabel('f  [Hz]')
% ylabel('|FRF6|  [m]')
% subplot(212),semilogx(f, 180/pi*unwrap(angle(FRF6)))
% grid on
% xlim([f(1) f(end)])
% xlabel('f  [Hz]')
% ylabel('\phi(FRF6)  [^\circ]')
% 
% x6 = lsim(sys_d6,input,t);
% 
% figure('Name','3d order, filtered, strictly proper - Simulation')
% subplot(211)
% hold on
% plot(t, output_z_filt,'g')
% plot(t, x6)
% title('3d order, filtered, strictly proper - Simulation VS Measurement')
% legend('pos_{z,meas}','pos_{z,sim}')
% xlabel('Time [s]')
% ylabel('Displacement [m]')
% axis tight
% subplot(212)
% plot(t,output_z_filt - x6)
% title('Difference between simulation and measurement')
% legend('pos_{z,meas}-pos_{z,sim}')
% xlabel('Time [s]')
% ylabel('Displacement [m]')
% axis tight
% 
% figure('Name','3d order, filtered, strictly proper - Pole Zero Map'),pzmap(sys_d6)
% 
% 
% % ---------------------
% %% FOURTH ORDER FITTING 
% % ---------------------
% 
% %% Filtering of the in- and output data using Butterworth filter
% [B, A] = butter(5, fcn); % order must be higher than order of system, 
%                             % adjust cut-off frquency to be higher than 
%                             % highest eigenfrequency of the system
%                              
% % input filtering                           
% input_filt = filter(B, A, input);
% % output filtering
% output_z_filt = filter(B, A, output_z);
% 
% figure('Name','4th order Butt. filtered Input & Measurement')
% subplot(211),plot(t,input,t,input_filt),title('Input filtered')
% legend('raw input', 'filtered input')
% subplot(212),plot(time, output_z, time, output_z_filt),title('pos_{z,filt}')
% legend('raw measurement', 'filtered measurement')
% 
% 
% %% With Butterworth filtering of in- and output.
% % --- STRICTLY PROPER [order(N) < order(D)]
% %
% %           Y(z)       b3*z^3 + b2*z^2 + b1*z + b0
% %   G(z) = ----- = -----------------------------------
% %           U(z)    z^4 + a3*z^3 + a2*z^2 + a1*z + a0
% %
% % y[k] = -a3*y[k-1]-a2*y[k-2]-a1*y[k-3]-a0*y[k-4]+b3*u[k-1]+b2*u[k-2]+b1*u[k-3]+b1*u[k-4]
% %
% % theta = [a3 a2 a1 a0 b3 b2 b1 b0]
% 
% y7 = output_z_filt(5:end);
% Phi7 = [-output_z_filt(4:end-1), -output_z_filt(3:end-2), -output_z_filt(2:end-3), -output_z_filt(1:end-4), input_filt(4:end-1), input_filt(3:end-2), input_filt(2:end-3), input_filt(1:end-4)];
% theta7 = Phi7\y7;
% 
% B7 = [theta7(5), theta7(6), theta7(7), theta7(8)];
% A7 = [1, theta7(1) theta7(2) theta7(3) theta7(4)];
% 
% sys_d7 = tf(B7, A7, Ts);
% 
% FRF7 = squeeze(freqresp(sys_d7,2*pi*f));
% 
% figure('Name','4th order, filtered, strictly proper - Freq. Resp.'), subplot(211)
% semilogx(f, 20*log10(abs(FRF6)))
% grid on
% xlim([f(1) f(end)])
% xlabel('f  [Hz]')
% ylabel('|FRF7|  [m]')
% subplot(212),semilogx(f, 180/pi*unwrap(angle(FRF6)))
% grid on
% xlim([f(1) f(end)])
% xlabel('f  [Hz]')
% ylabel('\phi(FRF7)  [^\circ]')
% 
% x7 = lsim(sys_d7,input,t);
% 
% figure('Name','4th order, filtered, strictly proper - Simulation')
% subplot(211)
% hold on
% plot(t, output_z_filt,'g')
% plot(t, x7)
% title('4th order, filtered, strictly proper - Simulation VS Measurement')
% legend('pos_{z,meas}','pos_{z,sim}')
% xlabel('Time [s]')
% ylabel('Displacement [m]')
% axis tight
% subplot(212)
% plot(t,output_z_filt - x7)
% title('Difference between simulation and measurement')
% legend('pos_{z,meas}-pos_{z,sim}')
% xlabel('Time [s]')
% ylabel('Displacement [m]')
% axis tight
% 
% figure('Name','4th order, filtered, strictly proper - Pole Zero Map'),pzmap(sys_d7)
% 
% 
% 


%% ------------------
% %  Comparison of fits
% %  ------------------
% figure
% plot(t,[output_z ...
%         x1 x2 x0 x3 x4 x5 x6 x7])
%         %x1 x2]) 
% legend('measurement',...
%         '2nd - no filter - proper',...
%         '2nd - filter - proper',...
%         '2nd - filter - strictly proper',...
%         '3d - no filter - proper',...
%         '3d - filter - proper',...
%         '3d - no filter - strictly proper',...
%         '3d - filter - strictly proper',...
%         '4th - filter - strictly proper')
% 
%  
%% Find crossover frequency of best fit
[error, index] = min(abs(20*log10(abs(FRF0(1:end-100)))));
f0 = f(index);

fprintf('f0z: %d \n', f0)

     
%% continuous time transfer function
sys_c0 = d2c(sys_d0, 'matched')


%% Save result (transfer function)
%save('HVJ_x_cont','sys_c2nd')

