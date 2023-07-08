%% Robotic forced vibration simulation [Robot KUKA + ode45]
clc;clear;% close all;
% In this program function [ode45] is used to simulate robotic EE chatter
tic

%% Robot import
Robot_Model % Robot Importation

%% Simulation Parameters
% Simualation time/excitation parameters
Simu_dt = 0.001; % [s]
Simu_num = 5000; % [scalar] observed time steps
Simu_E = 2000; % [scalar] excited time steps
Excit_base = [1,0,0]; % [scalar] excite distribution %%%%%%%%%%%%%%%%%%%%%%【】
% for example, if excit_base = [0 1 0], the excitation exsits only in Y
% direction.
Excit_def = 0.001*0; %initial deformation on robotic EE [scalar,m]%%%%%%%%%【】
Excit_value = 50; % amplitude of external exciting [scalar,N]%%%%%%%%%%%%%%【】
Excit_f = 100; % frequency of external excit [scalar,Hz]%%%%%%%%%%%%%%%%%%%【】
Excit_w = 1/Excit_f; % period of external excit [scalar,rad/s]

% Original point of trajectory 
P_TCP_stat = [1.5,0,1]; % [row,m] 
P_TCP_diff = [1.5 + Excit_def,0,1]; % [row,m]

%% Register definition
% Register for kinematics info [col, rad rad/s rad^2/s]
q_traj = zeros(6,Simu_num);
qd_traj = zeros(6,Simu_num);

TCP_traj = zeros(Simu_num,3);

% Register for EE position info [row, m]
EE_traj = zeros(Simu_num,3);

% Register for Excitation [row,N]
EF_traj = zeros(Simu_num,3);


%% Initial Kinematic info 
% Origin motion [col,rad rad/s rad/s^2]
stat_q = ikine(Robot_KUKA,transl(P_TCP_stat))';   
stat_qd = zeros(6,1);

ini_q = ikine(Robot_KUKA,transl(P_TCP_diff))';
ini_qd = ones(6,1)*0; 

% Robotic dynamic model generation
% Jacobian matrix [6*6 matrix]
Jacb = Robot_KUKA.jacob0(stat_q);
% Components in linearity equation of motion [6*6 matrix]
M_zero = Robot_KUKA.inertia(stat_q.'); % Mass matrix
C_zero = Robot_D; % Linearised Damping matrix
K_zero = Robot_K; % Linearised Stiffness matrix

Info.M = M_zero; % Define struct array, used in function [ode45][struct array]
Info.C = C_zero;
Info.K = K_zero;

%% Dynamics Simulation
for i = 1:Simu_num
    % info storage. 
    % At the starting step, initial info is loaded, in following steps,
    % info comes from previous iteration.
    if i == 1
        q_traj(:,i) = ini_q; % [col,rad]
        qd_traj(:,i) = ini_qd; % [col,rad/s]
        q = ini_q;  % [col,rad]
        qd = ini_qd; % [col,rad/s]
        EE_traj(i,:) = P_TCP_diff-P_TCP_stat; % [row, m]
    else
        q_traj(:,i) = q; % [col,rad]
        qd_traj(:,i) = qd; % [col,rad/s]
        EE_traj(i,:) = transl(fkine(Robot_KUKA,q_traj(:,i)))-P_TCP_stat; % [row, m]
    end

    TCP_traj(i,:) = transl(fkine(Robot_KUKA,q));

    % Excitation [col,Nm] 
    if i < Simu_E
        Excit = Excit_base*sin(i*Excit_w)*Excit_value; % [row,N]
    else
        Excit = Excit_base*0; % [row,N]
    end
    EF_traj(i,:) = Excit; % [row,N]
    Jacb = Jacb(1:3,:); % [3*6 matrix]
    Excit = Jacb.'*Excit.'; % [col,Nm]
    Info.F = Excit; % Define struct array, used in function [ode45][col,Nm]
   

    % Robot dynamics solution
    [q,qd] = Robot_Dynode45(i,Simu_dt,q-stat_q,qd,Info);
    q = q + stat_q;
    qd = qd;

    % Cycle index updating and indication
    if mod(i,100) == 0
        disp(strcat('Cycle:',num2str(i),' is over'))
        % disp(err)
    end
end

% Data visulization
fig1 = figure;
hold on
plot(EE_traj(1:end,1))
plot(EE_traj(1:end,2))
plot(EE_traj(1:end,3))
% plot([Simu_E,Simu_E],[-5e-4,5e-4],'r')% draw split line
% 
fig2 = figure;
hold on
plot(EF_traj(1:end,1))

toc

%% Signal time-frequency domain analysis based on wavelet transfer
fs = 1/Simu_dt; % sampling frequency [scalar,Hz]
t=0:Simu_dt:Simu_num*Simu_dt; % time axis of data [row,s] %%%%%%%%%%%%%%%%%【】
s=EE_traj(:,1).'; % signal [row] %%%%%%%%%%%【】
% 连续小波变换
wavename='cmor3-3'; % wavelet name [str]
totalscal=256*2; % 尺度序列的长度，决定绘图分辨率 [scalar] %%%%%%%%%%%%%%%%%%%【】
Fc=centfrq(wavename); % 小波的中心频率 [scalar,Hz]
c=2*Fc*totalscal;
scals=c./(1:totalscal);
f=scal2frq(scals,wavename,1/fs); % 将尺度转换为频率
coefs=cwt(s,scals,wavename); % 求连续小波系数

figure
imagesc(t,f,abs(coefs));
set(gca,'YDir','normal')
colorbar;
xlabel('Simulation time (s)');
ylabel('Frequency (Hz)');
title('Time-frequency analysis of vibration');
ylim([0,80])
