%% Simulation CuttingTool half down
% Cutting force on Cutting tool is simulated in this program
% unit: length [mm]; angle = phase [rad]
% [version 1] 20220706 对原始文件进行改造，得到本程序
% [version 2] 20220711 修改嵌套循环逻辑，经验证结果不变，运行时常微微缩短
% [version 3] 20220717 修复已知问题，使用最新chip thickness计算程序进行更新
% [version 4] 20220722 按照机器人铣削耦合模型修改结果修改本程序
% [version 5] 20220824 机器人铣削耦合动力学仿真结束后，修正本程序
clc;clear;close all
tic % time counter

%% Parameters Definition and Display
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%------------------Milling spindle Definition----------------------------%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Properties of cutting tool 
R = 10/2; % radius of cutting tool, [scalar,mm,modify] %%%%%%%%%%%%%%%%%%%%【】
Helix = 30; % helix angle of cutting tool [scalar,deg,modify] %%%%%%%%%%%%%【】
Work = 0; % 0: half immersion (down milling) [scalar,0/1/2]
% if cutting tool works in [half immersion], each tooth works in 1/4 period
% if cutting tool works in [full immersion], each tooth works in 1/2 period
if Work == 0 || Work == 1;n_tip = 4;else;n_tip = 2;end

% Machining and motion parameters
% [W] and [N]: only one of them should be given, the other one can be
% calculated by known info.
% 1.[W] is given
% W = 527*2*pi/60; % angular velocity of cutting tool, [scalar,rad/s,modify]
% N = 60*W/2/pi; % = 527, spindle speed of cutting tool, [scalar,rpm]
% 2.[N] is given
N = 1980; % spindle speed of cutting tool, [scalar,rpm,modify] %%%%%%%%%%%%【】determined at 20221216
Ap = 1; % axial depth of cutting tool [scalar,mm,modify] %%%%%%%%%%%%%%%%%%【】 determined at 20221216
W = N*2*pi/60; % angular velocity of cutting tool, [scalar,rad/s]
T = 2*pi/W; % Rotation period, [scalar,s]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%---------------------Simulation Preparation-----------------------------%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% * Simulation Parameters about Time and Sampling 
% Attention! [n] means [num = number], all variables having [n] is num of
% time steps.
sim_desire_time = 1; % desired simulation time [s]%%%%%%%%%%%%%%%%%%%%%%%%%【】 determined at 20221216
sim_n_dt_period = 120; % number of time steps in period; [scalar, modify]%%【】 determined at 20221216
% Note! [sim_desire_time] is determined by user and measuring system;
% [sim_n_dt_period] is defined by sampling frequency of measuring system
% [f], their relationship is:
%                     [f] = [sim_n_dt_period]/[T]
%                     [sim_n_dt_period] = [f][T]
% determined at 20221216:[T] = 0.03s, when [sim_n_dt_period] = 180/90,
% frequency is 6000/3000 Hz, this parameters has been inputted in [DynoWare]
sim_n_start = 2; %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%【】
sim_n_dt_start = sim_n_dt_period * sim_n_start; 
% number of time steps required before simulation [scalar]
sim_n_period = ceil((sim_desire_time + sim_n_start*T)/T);
% number of total simulation period, [scalar,s,modify]
% Note！the extra [2*T] is cuased by [Notch machining], which is defined 
% by [sim_n_start]
% * Derived Parameters
sim_T = T*sim_n_period; 
% Simulation time duration, [scalar,s]
[sim_dt,sim_n_dt_period] = Cut_Simudt(W,n_tip,sim_n_dt_period); 
% Length of every discrete time step, [scalar,s]
% number of time steps in a rotation period [scalar,s]
sim_n_dt_total = sim_n_dt_period*sim_n_period ; 
% number of time steps in simulation duration [scalar]
sim_n_dt_work = sim_n_dt_period/n_tip; 
% number of time steps, in which a tip is in working, [scalar]

% * Simulation Parameters about Cutting tool
% [slice_n] and [slice_dphase]:only one of them should be given, the other 
% one can be defined by given info
% 1.[slice_n] is given
% slice_n = 51;% num of slices in cutting tool [scalar,modify]
% slice_dphase = (Ap*tand(Helix)/R)/slice_n; % phase between slice[scal,rad] 
% between 2 neighboring slices.
% 2.[slice_dphase] is given
slice_dphase = 2*pi/1080; % phase difference between each 2 slices [scalar,rad] .
slice_n = ceil(Ap*tand(Helix)*360/(R*2*pi)); % num of slices [scalar]
% * Derived Parameters
dz = Ap/slice_n; % height of slice [scalar,mm]
ds = dz/cosd(Helix); % length of cutting edge [scalar,mm]
CFC = Cut_ForceCalculator(); % Cutting force coefficients[obj,modify]

% * Simulation Parameters about machining tool
% During milling process,robot's trajectory is a uniform linear motion 
% along x-axis, feed speed is constant.
f_mm = 2.4; % feed speed [scalar,mm/s,modify]%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%【】 determined at 20221216
f_m = f_mm/1000; % [scalar,m/s,useless setttings]
TCP_xy_mm = [0,0]'; % initial position of cutting tool [col,mm,useless setttings]
TCP_xy_m = TCP_xy_mm/1000; % [col,m,useless setttings]

% Machining condition display
disp(strcat('machining condition: spindle speed n = ',num2str(N), ...
            ' [mm]; ','axial depth ap = ',num2str(Ap),' [mm]'))


%% Simulation Initialization
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %-------------------------Initialization -------------------------------%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% * Registeer of TCP trajectory
% Real trajectory of TCP (cutting tool) is stored here.
Traj_TCP = zeros(2,sim_n_dt_total);

% * Register of Trajectory data in a period
% Trajectory data used to calculate chip thickness, store only effect
% working points in one period, size of register is 4×[sim_n_dt_period]. 
% Row1:phase; Row2+3:position[x,y]; R4:work_sign
Traj_period_T1 = zeros(4*slice_n,sim_n_dt_work);
Traj_period_T2 = zeros(4*slice_n,sim_n_dt_work);
Traj_period_T3 = zeros(4*slice_n,sim_n_dt_work);
Traj_period_T4 = zeros(4*slice_n,sim_n_dt_work);

% * Register of Trajectory data for observation
% All data points in trajectory are stored here.
Traj_T1 = zeros(4*slice_n,sim_n_dt_total);
Traj_T2 = zeros(4*slice_n,sim_n_dt_total);
Traj_T3 = zeros(4*slice_n,sim_n_dt_total);
Traj_T4 = zeros(4*slice_n,sim_n_dt_total);

% * Register of Chip thicknesses of each teeth
% store chip thickness of Teeth i in the whole simulated process
% each register is a n-d vector.
Chip_thick_T1 = zeros(1*slice_n,sim_n_dt_total); 
Chip_thick_T2 = zeros(1*slice_n,sim_n_dt_total); 
Chip_thick_T3 = zeros(1*slice_n,sim_n_dt_total); 
Chip_thick_T4 = zeros(1*slice_n,sim_n_dt_total); 

% * Registers for cutting force of each teeth in moving frame TRA
% store cutting force of Teeth i in the whole simulated process, cutting
% force here include [tangential,radial and axial forces]
% each register is a 3*n matrix.
CutF_TRA_T1 = zeros(3*slice_n,sim_n_dt_total); 
CutF_TRA_T2 = zeros(3*slice_n,sim_n_dt_total); 
CutF_TRA_T3 = zeros(3*slice_n,sim_n_dt_total); 
CutF_TRA_T4 = zeros(3*slice_n,sim_n_dt_total); 

% * Registers for cutting force of each teeth in frame TCP
% store cutting force of Teeth i in the whole simulated process, cutting
% force here are projected in frame TCP
% each register is a 3*n matrix.
CutF_TCP_T1 = zeros(3*slice_n,sim_n_dt_total); 
CutF_TCP_T2 = zeros(3*slice_n,sim_n_dt_total); 
CutF_TCP_T3 = zeros(3*slice_n,sim_n_dt_total); 
CutF_TCP_T4 = zeros(3*slice_n,sim_n_dt_total); 

% Register for global cutting force
CutF_TCP = zeros(3,sim_n_dt_total);

%% Slices generation
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %---------------------------Slice Generation ---------------------------%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Cutting tool slice definition
% input: [C_R] radius of cutting tool [mm]
%        [C_w] angular velocity of cutting force [rad/s]
%        [C_f] feed speed of cutting tool [mm/s]
%        [C_dt] length of time step [s]
%        [C_work_modal] current work modal [0/1/2],generally 0
%        [C_angle] initial phase of cutting tool [rad]
%        [C_centre] coordinate of cutter, [col,mm]
%        [n_TStep_inCycle] time steps'num in cycle [scalar]
for i = 1:slice_n
    C_theta = -(i-1)*slice_dphase; % Attention!
    CutTool_s(i) = Cut_SliceClass(R,W,f_mm,sim_dt,Work,C_theta,TCP_xy_mm,sim_n_dt_period);  
end

%% Simulation conduction
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %-----------------------Large Loop Simulation --------------------------%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1:1:sim_n_dt_total
    % i: number of simulation time step
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %----------------Current real trajectory generation-----------------%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Real trajectory of each teeth in current instant (instant 1) of time
    % step [i] is captured in this part. Trajectories of all [slice_n]
    % slices are stored respectively in a for loop.
    for j = 1: slice_n
        % i: number of simulation time step
        % j: number of cutting slices

        % * Determine the [row index]
        % Trajectory and cutting force of each cutting slice occupy only 4 
        % and 3 rows, but information of all slices are stored in a global
        % matrix register, it is necessary to determine the position of []
        % in this case is easy to determine.
        % To store trajectory and cutting force information of all cutting 
        % slice, totally [4*slice_n] and [3*slice_n] register are
        % necessary. In this case, [row index] of each needs special design.
        k4 = (j-1)*4; % Counte basis of slice j in [Trajectory register]
        k3 = (j-1)*3; % Counte basis of slice j in  [CutF register]

        % * Trajectory of teeth 1
        n_past = CutTool_s(j).S_num_past1; % num of past points [scalar]
        n_cout = i-n_past-sim_n_dt_work; % current point index [scalar]
        % [n_cout] is start-counting point, it comprises complex components
        % 1. [i] number of current data points
        % 2. [simu_n_dt_work] working time steps in a period, tooth i start
        %    working after time step [simu_n_dt_work]*i. This conclusion
        %    considers initial empty 1/4 period.
        % 3. [n_past] past points in previous periods
        % 4. [-1] to ensure that length of [Traj_period] is [sim_n_dt_period]
        if n_cout > 0
            if CutTool_s(j).T1_working == 1
            Traj_period_T1(k4+1,n_cout) = CutTool_s(j).T1_current_angle;
            Traj_period_T1(k4+2,n_cout) = CutTool_s(j).T1_current_tip_p(1);
            Traj_period_T1(k4+3,n_cout) = CutTool_s(j).T1_current_tip_p(2);
            Traj_period_T1(k4+4,n_cout) = CutTool_s(j).T1_working;
            Traj_T1(k4+1,i) = CutTool_s(j).T1_current_angle;
            Traj_T1(k4+2,i) = CutTool_s(j).T1_current_tip_p(1);
            Traj_T1(k4+3,i) = CutTool_s(j).T1_current_tip_p(2);
            Traj_T1(k4+4,i) = CutTool_s(j).T1_working;
            end
        end

        % * Trajectory of tooth 2
        n_past = CutTool_s(j).S_num_past2;
        n_cout = i-n_past-sim_n_dt_work*2;
        if n_cout > 0
            if CutTool_s(j).T2_working == 1
            Traj_period_T2(k4+1,n_cout) = CutTool_s(j).T2_current_angle;
            Traj_period_T2(k4+2,n_cout) = CutTool_s(j).T2_current_tip_p(1);
            Traj_period_T2(k4+3,n_cout) = CutTool_s(j).T2_current_tip_p(2);
            Traj_period_T2(k4+4,n_cout) = CutTool_s(j).T2_working;
            Traj_T2(k4+1,i) = CutTool_s(j).T2_current_angle;
            Traj_T2(k4+2,i) = CutTool_s(j).T2_current_tip_p(1);
            Traj_T2(k4+3,i) = CutTool_s(j).T2_current_tip_p(2);
            Traj_T2(k4+4,i) = CutTool_s(j).T2_working;
            end
        end

        % * Trajectory of teeth 3
        n_past = CutTool_s(j).S_num_past3;
        n_cout = i-n_past-sim_n_dt_work*3;
        if n_cout > 0
            if CutTool_s(j).T3_working == 1
            Traj_period_T3(k4+1,n_cout) = CutTool_s(j).T3_current_angle;
            Traj_period_T3(k4+2,n_cout) = CutTool_s(j).T3_current_tip_p(1);
            Traj_period_T3(k4+3,n_cout) = CutTool_s(j).T3_current_tip_p(2);
            Traj_period_T3(k4+4,n_cout) = CutTool_s(j).T3_working;
            Traj_T3(k4+1,i) = CutTool_s(j).T3_current_angle;
            Traj_T3(k4+2,i) = CutTool_s(j).T3_current_tip_p(1);
            Traj_T3(k4+3,i) = CutTool_s(j).T3_current_tip_p(2);
            Traj_T3(k4+4,i) = CutTool_s(j).T3_working;
            end
        end

        % * Trajectory of teeth 4
        n_past = CutTool_s(j).S_num_past4;
        n_cout = i-n_past-sim_n_dt_work*4;
        if n_cout > 0
            if CutTool_s(j).T4_working == 1
            Traj_period_T4(k4+1,n_cout) = CutTool_s(j).T4_current_angle;
            Traj_period_T4(k4+2,n_cout) = CutTool_s(j).T4_current_tip_p(1);
            Traj_period_T4(k4+3,n_cout) = CutTool_s(j).T4_current_tip_p(2);
            Traj_period_T4(k4+4,n_cout) = CutTool_s(j).T4_working;
            Traj_T4(k4+1,i) = CutTool_s(j).T4_current_angle;
            Traj_T4(k4+2,i) = CutTool_s(j).T4_current_tip_p(1);
            Traj_T4(k4+3,i) = CutTool_s(j).T4_current_tip_p(2);
            Traj_T4(k4+4,i) = CutTool_s(j).T4_working;
            end
        end
    end

    % Current real TCP trajectory store
    Traj_TCP(:,i) = CutTool_s(1).C_current_Centre_p/1000; % [col,m]

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %-------------------Feed in time step updating-----------------------%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % At predict instant (instant 2) of time step [i], cutting force is
    % predicted and corrected by a [while loop], a suitable [feed] in both 
    % x and y directions are also calculated during this procedure.
    if i <= sim_n_dt_start 
        % if index <= simu_n_dt_start, the program is perparing for cutting
        % robot during this process is seen as rigid.
        disp(strcat("Cycle Nr.",num2str(i)," in ",num2str(sim_n_dt_total),... 
             " is over, but simulation hasn't started yet. "))
    else
        dw = CutTool_s.C_dw; 
        % Phase variation in [sim_dt][scalar,rad]
        CutF_TCP_2 = zeros(3,1); 
        % Sum of cutting force, used for cutting force traversal [col,N]
        for j = 1: slice_n
            % i: number of simulation time step
            % j：number of cutting slices
            % * Determine the [row index]
            k4 = (j-1)*4; % Counte basis of slice k in [Trajectory register]
            k3 = (j-1)*3; % Counte basis of slice k in  [CutF register]

            [C_T1,work_r1,~] = Cut_ChipThickness(CutTool_s(j).T1_predict_tip_p, ...
                                           Traj_period_T4(k4+2,1:end), ...
                                           Traj_period_T4(k4+3,1:end), ...
                                           TCP_xy_mm, ...
                                           CutTool_s(j).T1_working, ...
                                           dw);
            Chip_thick_T1(j,i+1) = C_T1; % Chipthickness of teeth 1 [scalar mm]
            CutTool_s(j).T1_working = work_r1; % Updated work state of teeth 1 [scalar 0/1]
            [CutF_TRA_T1(k3+1:k3+3,i+1),CutF_TCP_T1(k3+1:k3+3,i+1)] = Cutting_Force(CFC, ...
                                           C_T1 * CutTool_s(j).T1_working,dz,ds, ...
                                           CutTool_s(j).T1_predict_angle, ...
                                           CutTool_s(j).T1_working);% [col,N]

            [C_T2,work_r2,~] = Cut_ChipThickness(CutTool_s(j).T2_predict_tip_p, ...
                                           Traj_period_T1(k4+2,1:end), ...
                                           Traj_period_T1(k4+3,1:end), ...
                                           TCP_xy_mm, ...
                                           CutTool_s(j).T2_working, ...
                                           dw);
            Chip_thick_T2(j,i+1) = C_T2;  % Chipthickness of teeth 2 [scalar mm]
            CutTool_s(j).T2_working = work_r2; % Updated work state of teeth 2 [scalar 0/1]
            [CutF_TRA_T2(k3+1:k3+3,i+1),CutF_TCP_T2(k3+1:k3+3,i+1)] = Cutting_Force(CFC, ...
                                           C_T2 * CutTool_s(j).T2_working,dz,ds, ...
                                           CutTool_s(j).T2_predict_angle, ...
                                           CutTool_s(j).T2_working);% [col,N]

            [C_T3,work_r3,~] = Cut_ChipThickness(CutTool_s(j).T3_predict_tip_p, ...
                                           Traj_period_T2(k4+2,1:end), ...
                                           Traj_period_T2(k4+3,1:end), ...
                                           TCP_xy_mm, ...
                                           CutTool_s(j).T3_working, ...
                                           dw);
            Chip_thick_T3(j,i+1) = C_T3; % Chipthickness of teeth 3 [scalar mm]
            CutTool_s(j).T3_working = work_r3; % Updated work state of teeth 3 [scalar 0/1]
            [CutF_TRA_T3(k3+1:k3+3,i+1),CutF_TCP_T3(k3+1:k3+3,i+1)] = Cutting_Force(CFC, ...
                                           C_T3 * CutTool_s(j).T3_working,dz,ds, ...
                                           CutTool_s(j).T3_predict_angle, ...
                                           CutTool_s(j).T3_working);% [col,N]

            [C_T4,work_r4,~] = Cut_ChipThickness(CutTool_s(j).T4_predict_tip_p, ...
                                           Traj_period_T3(k4+2,1:end), ...
                                           Traj_period_T3(k4+3,1:end), ...
                                           TCP_xy_mm, ...
                                           CutTool_s(j).T4_working, ...
                                           dw);
            Chip_thick_T4(j,i+1) = C_T4; % Chipthickness of teeth 4 [scalar mm]
            CutTool_s(j).T4_working = work_r4; % Updated work state of teeth 4 [scalar 0/1]
            [CutF_TRA_T4(k3+1:k3+3,i+1),CutF_TCP_T4(k3+1:k3+3,i+1)] = Cutting_Force(CFC, ...
                                           C_T4 * CutTool_s(j).T4_working,dz,ds, ...
                                           CutTool_s(j).T4_predict_angle, ...
                                           CutTool_s(j).T4_working);% [col,N]

            % Resultant cutting force in frame TCP in instant 2 [col,N]
            CutF_TCP_2 = CutF_TCP_2 + CutF_TCP_T1(k3+1:k3+3,i) + ...
                           CutF_TCP_T2(k3+1:k3+3,i) +  CutF_TCP_T3(k3+1:k3+3,i) + ...
                           CutF_TCP_T4(k3+1:k3+3,i); % [col,N]
        end
        % Resultant cutting force in frame TCP in instant 2 [col,N]
        CutF_TCP(:,i+1) =  CutF_TCP_2;
        % disp(CutF_TCP_2) % Cutting force showcase

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %----------Traversal info print------------%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        disp(strcat("Cycle Nr.",num2str(i)," in ",num2str(sim_n_dt_total)," is over")) 
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %---------------------------Info updating----------------------------%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Information updating for next instant
    % Predict parameters in instant 2 of time step i step becomes current
    % parameters in instant 1 of time step i+1; predict parameters in time
    % step i+1 will be updated.
    for j = 1:slice_n
        CutTool_s(j).Tool_info_updating;
    end
end

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%-------------------------Result Construction --------------------------%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CutF_TCP_slice = CutF_TCP_T1 + CutF_TCP_T2 + CutF_TCP_T3 + CutF_TCP_T4;
% CutF_TCP = zeros(3,sim_n_dt_total);
% for m = 1:slice_n
%     m3 = (m-1)*3+1; % Counting basis for register [cutF_TCP_slice]
%     CutF_TCP = CutF_TCP + CutF_TCP_slice(m3:m3+2,:);
% end
% Consequence print
n_rotation = CutTool_s(1).S_num_rotation1;
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp(strcat('% Totally:',num2str(n_rotation),' turns are simulated. %'))
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')

toc % time counter

% %% this part is used to generate data for cutting force coefficient 
% % identification.
% CutF_TCP_process = CutF_TCP(:,sim_n_dt_start+2:end);
% % cutting force calculated in simulation is cutting force projected in 
% % [inertia frame], but the needed data is observed in [machining plane 
% % frame]. According to their positive directions, data in Y and Z
% % directions should be added 'minus(-)'
% 
% L = length(CutF_TCP_process); % L = 5940
% Fc = zeros(L,1);
% for i = 1:L
%     Fc(3*i-2) = CutF_TCP_process(1,i);
%     Fc(3*i-1) = -CutF_TCP_process(2,i);
%     Fc(3*i-0) = -CutF_TCP_process(3,i);
% end
% plot([Fc(1:3:end,:),Fc(2:3:end,:),Fc(3:3:end,:)])

%% Observation of chip thickness of each slice
fig = figure;
hold on
for i = 1:slice_n
plot(Chip_thick_T1(i,:),'o')
plot(Chip_thick_T2(i,:),'o')
plot(Chip_thick_T3(i,:),'o')
plot(Chip_thick_T4(i,:),'o')
end
title('Chip thickness')
