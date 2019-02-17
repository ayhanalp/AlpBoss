% PID controller interpretation

% COLORS FOR FIGURES:
% Blue
% 'Color', [0.3010, 0.7450, 0.9330], 'LineWidth',2.5
% Yellow
% 'Color', [0.9290, 0.6940, 0.1250], 'LineWidth',2.5
% Red
% 'Color', [0.6350, 0.0780, 0.1840], 'LineWidth',2.5

clear variables
close all
clc


%% Identify models
run identify_params


%% Calculate PID controller parameters
fprintf('\n== Start PID controller parameter calculation =\n')
options.figures = true;
options.prints = false;

% Desired phase margin
PM_des = 45;

% Function calls
fprintf('\n----------------- x direction -----------------\n')
[xsys_cl,xPIDparams] = PID_design(xmodel, PM_des, options);
fprintf('\n----------------- y direction -----------------\n')
[ysys_cl,yPIDparams] = PID_design(ymodel, PM_des, options);

% -------------------------------------------------------------------------
% fprintf('\n----------------- z direction -----------------\n')
% [zsys_cl,zPIDparams] = PID_design(zmodel, PM_des, options);
% NOTE - z-direction proportional controller sufficient. 
%        This procedure doesn't work for z.


fprintf('\n======== PID controller design finished =======\n')


%% ========================================================================
%                               Main function
%  ========================================================================

function [sys_cl,PIDparams] = PID_design(model, PM, options)

G = model.tf_pos;

w = logspace(0,2,10^3);
s = tf('s');

[~,Gphase,~] = bode(G,w);
Gphase = squeeze(Gphase);

% figure('Name','Bode & margins of uncompensated system')
% margin(G);

%% Determine Td
%  Td chosen as function of PM, taking into account anticipated lag of 10 -
%  15 ° due to integrator.
%  c: correction, not exactly 90° added by lead, not exactly 15° by lag.
c = 6;
phi = -180 + PM - 90 + 15 + c;
% crossover frequency
w_c = interp1(Gphase,w,phi);

Td = 10/w_c;
Ti = 3.73/w_c;

%% D and I part separately
% D(s) = K * (1 + Td*s) * (1 + 1/(s Ti))
% Dd(s) = (1 + Td*s), Di(s) = ((1 + 1/(s Ti)), D(s) = K*Dd(s)*Di(s)
Dd = (1 + Td*s);
Di = (1 + 1/(Ti*s));

%% Determine K
mag = abs((evalfr(G*Dd*Di,1j*w_c)));
K = 1/(mag);

%% Resulting PID-controller
D = K*Dd*Di;

PIDparams.Td = Td;
PIDparams.Ti = Ti;
PIDparams.K = K;

PIDparams.Kp = K*(1+Td/Ti);
PIDparams.Ki = K/Ti;
PIDparams.Kd = K*Td;

% Display calculated parameters in command window
if options.prints
    fprintf('\nPID parameters\n')
    disp(PIDparams)
end


%% Visualisation & Command window information

sys_ol = G*D;
sys_cl = G*D/(1+G*D);

[Gm,Pm,Wcg,Wcp] = margin(sys_ol);

if options.figures
    figure('Name', 'PID controller bode')
    bode(D)
    
    figure('Name','Margins of closed loop system for varying control parameter K')
    hold on
    margin(G)
    margin(sys_ol);
    
    % Some dirty manipulating to choose color and line thickness:
    bodeplot(G,'b');
    lineHandle = findobj(gcf,'Type','line','-and','Color','b');
    set(lineHandle,'Color',[0.3010, 0.7450, 0.9330]);
    bodeplot(sys_ol,'b');
    lineHandle = findobj(gcf,'Type','line','-and','Color','b');
    set(lineHandle,'Color',[0.6350, 0.0780, 0.1840])
    
    set(findall(gcf,'type','line'),'linewidth',2)
    
    h = flipud(findobj(gcf,'type','Axes'));
    hl1 = flipud(findobj(h(1),'type','Line'));
%     hl2 = flipud(findobj(h(2),'type','Line'));
 
    legend(h(1),hl1(3:4),'Uncompensated','PID Compensated')
end

% Display margins, poles in command window
if options.prints
    fprintf('Phase margin, Gain margin, Gain w_c, Phase w_c\n')
    disp([Pm,Gm,Wcg,Wcp])

    fprintf('\nPoles of closed loop system:\n')
    disp(pole(sys_cl))

end

% Plot pzmap
if options.figures
    figure('Name','Pzmap of closed loop system for varying control parameter K')
    pzplot(sys_cl)
end


end

