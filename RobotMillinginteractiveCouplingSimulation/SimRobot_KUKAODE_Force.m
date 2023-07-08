%% Robotic forced vibration simulation [Robot KUKA + ode45]
clc;clear;% close all;
% In this program function [ode45] is used to simulate robotic EE chatter
tic % timing starting

%% Robot import
Robot_Model % Robot Importation

%% Simulation Preparation
% Simualation time/excitation parameters
simu_dt = 0.001; % [s]
simu_num = 2000; % [scalar] observed time steps
excit_base = [0,1,0]; % [scalar] excite distribution

% Simulation trajectory parameters
f_mm = 0.4; % [scalar,mm/s] feed speed
f_m = f_mm/1000; % [scalar,m/s] feed speed

% Simulation original point of trajectory 
TCP_desi_ori = [1.5,0,1]; % [col,m] desired origin of trajectory
TCP_real_ori = [1.5,0,1]; % [col,m] real origin of trajectory,
% commonly the same as [TCP_desi_ori]

%% Register definition
% Register for robotic posture deformation and its derivative [col, rad rad/s rad^2/s]
vq_traj = zeros(6,simu_num);
vqd_traj = zeros(6,simu_num);

% Register for robotic desired posture [col, rad]
desi_q_traj = zeros(6,simu_num);
desi_qd_traj = zeros(6,simu_num);

% Register for robotic simulated posture [col,rad]
real_q_traj = zeros(6,simu_num);
real_qd_traj = zeros(6,simu_num);

% Register for EE position info [row, m]
EE_traj = zeros(simu_num,3);

% Register for Excitation [row,N]
EF_traj = zeros(simu_num,3);

%% Dynamics Simulation
for i = 1:simu_num
    %%%%%%%%%%%%%%%%%% Current motion define and store %%%%%%%%%%%%%%%%%%
    % All parameters come from initial values and iterations
    if i == 1
        % Initial desired and real TCP [row,m]
        desi_1_TCP = TCP_desi_ori; % [row,m]
        real_1_TCP = TCP_real_ori; % [row,m]
        % Real posture 1 and its generalized velocity [col, rad rad/s]
        [real_1_q,real_1_qd] = Robot_LTraj(Robot_KUKA,real_1_TCP,f_m,Robot_R);
        % Desired posture 1 and its generalized velocity [col,rad rad/s]
        [desi_1_q,desi_1_qd] = Robot_LTraj(Robot_KUKA,desi_1_TCP,f_m,Robot_R);
        % Posture deformation and its generalized velocity [col,rad rad/s]
        vq = real_1_q - desi_1_q ;  % [col,rad]
        vqd = real_1_qd - desi_1_qd; % [col,rad/s]
    else
        desi_1_TCP = desi_2_TCP; % [row,m]
        real_1_TCP = real_2_TCP; % [row,m]
        desi_1_q = desi_2_q; % [col,rad]
        desi_1_qd = desi_2_qd; % [col,rad/s]
        vq = vq; % [col,rad]
        vqd = vqd; % [col,rad/s]
    end

    EE_traj(i,:) = real_1_TCP; % [row, m]
    desi_q_traj(:,i) = desi_1_q; % [col,rad] desired motion
    desi_qd_traj(:,i) = desi_1_qd; % [col,rad/s]
    vq_traj(:,i) = vq; % [col,rad] posture deformation
    vqd_traj(:,i) = vqd; % [col,rad/s]
    % real_q_traj(:,i) = real_1_q; % [col,rad] real motion
    % real_qd_traj(:,i) = real_1_qd; % [col,rad]

    %%%%%%%%%%%%%%%%% Robotic dynamic model generation %%%%%%%%%%%%%%%%%%
    % Jacobian matrix [6*6 matrix]
    Jacb = Robot_KUKA.jacob0(desi_1_q);
    % Components in linearity equation of motion [6*6 matrix]
    M_zero = Robot_KUKA.inertia(desi_1_q.'); % Mass matrix
    C_zero = Robot_D; % Linearised Damping matrix
    K_zero = Robot_K; % Linearised Stiffness matrix
    % Struct array used in function [ode45]
    Info.M = M_zero; % [struct]
    Info.C = C_zero; % [struct]
    Info.K = K_zero; % [struct]

    %%%%%%%%%%%%%%%%%%%%%% Predict motion define %%%%%%%%%%%%%%%%%%%%%%%%
    % Desired posture 2 and its generalized velocity [row,rad rad/s]
    desi_2_TCP = desi_1_TCP + [f_m*simu_dt,0,0];
    [desi_2_q,desi_2_qd] = Robot_LTraj(Robot_KUKA,desi_2_TCP,f_m,Robot_R);

    %%%%%%%%%%%%%%%%%%%%%% Excitation [col,Nm] %%%%%%%%%%%%%%%%%%%%%%%%%%
    Excit = excit_base*sin(i*0.01)*500; % [row,N] 
    EF_traj(i,:) = Excit; % [row,N]
    Jacb = Jacb(1:3,:); % [3*6 matrix]
    Excit = Jacb.'*Excit.'; % [col,Nm]
    Info.F = Excit; % Struct array used in function [ode45][col,Nm]

    %%%%%%%%%%%%%%%%%%%%% Robot dynamics solution %%%%%%%%%%%%%%%%%%%%%%%
%     tspan = [(i-1)*simu_dt,i*simu_dt]; % time interval in time step [row,s]
%     vQ_current = [vq;vqd]; % [col,rad rad/s 12d] Joint deformation!
%     opts = odeset('Mass',@(t,Q) mass(t,Q,Info),'InitialStep',simu_dt);
%     [~,vQ] = ode45(@(t,Q) robotdynamic(t,Q,Info),tspan,vQ_current,opts);
%     vq = vQ(end,1:6)'; % Attention! joint deformation!
%     vqd = vQ(end,7:12)';
    [vq,vqd] = Robot_Dynode45(i,simu_dt,vq,vqd,Info);

    %%%%%%%%%%%%%%%%%%%%%% Predict motion define %%%%%%%%%%%%%%%%%%%%%%%%
    real_2_q = desi_2_q + vq; % [col,rad]
    real_2_qd = desi_2_qd + vqd; % [col,rad/s]
    real_2_TCP = transl(Robot_KUKA.fkine(real_2_q));

    % %%%%%%%%%%%%% Cycle index updating and indication %%%%%%%%%%%%%%%%%
    if mod(i,10) == 0
        disp(strcat('Cycle:',num2str(i),' is over'))
        % disp(err)
    end
end

% Data visulization
fig1 = figure;
subplot(3,1,1)
plot(vq_traj(1,:))
subplot(3,1,2)
plot(vq_traj(2,:))
subplot(3,1,3)
plot(vq_traj(3,:))

fig2 = figure;
hold on
plot(EF_traj(1:end,2))

toc % timing endding

%%%%%%%%%%%%%%%%%%%% Function used in [ode45] %%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%Mass matrix: coefficient of ordinary derivative item %%%%%%%%%%
function MM = mass(t,Q,Info)
C = Info.C;
M = Info.M;
S = length(C);
Z = zeros(S,S);
MM = [C,M;
      M,Z];
end

%%%%%%%%%%%%%%%%%% Ordinary differential equation %%%%%%%%%%%%%%%%%%%%%%%
function dydt = robotdynamic(t,Q,Info)
F = Info.F;
K = Info.K;
M = Info.M;
S = length(M);
Z = zeros(S,S);
dydt = [F;zeros(S,1)]-[K,Z;Z,-M]*Q;
end




