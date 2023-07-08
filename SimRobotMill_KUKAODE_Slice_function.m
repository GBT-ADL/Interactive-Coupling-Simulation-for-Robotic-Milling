function name = SimRobotMill_KUKAODE_Slice_function(Spindle_speed,Axial_depth)

% Robotic milling procedure are simulated in this program. Only one cutting
% slice is considered here.
% [version 1] 20220630 统一修改m文件名称
% [version 2] 20220701 完成考虑机器人柔性的刀片铣削力编程v1
% [version 3] 20220702 修正已知bug，规划程序大修，包括加工平面坐标变换等,完成考虑机器人柔性的刀片铣削力编程v2
% [version 4] 20220703 修正已知bug，完成程序大修，梳理仿真思路，完成考虑机器人柔性的刀片铣削力编程v3
% [version 5] 20220704 修正已知bug，完成考虑机器人柔性的刀片铣削力编程vf
%             20220705 将机器人摩擦/重力补偿和切削力函数替换为类对象；根据基本假设对铣削力仿真进行增删修补；修正Robo_LTraj函数，进一步提高结果精度
% [version 6] 20220705 完全使用Residual form计算机器人动力学，重新编写程序，速度大大提升
% [version 7] 20220717 重写chip thickness算法，再从头编写机器人动力学逻辑
%             20220719 放弃速度更新，无法排除Simu_dt影响，仅使用Feed更新
% [version 8] 20220721 锁定机器人动力学Bug！驱动力矩无法及时追踪切削力变化！采用时间步差分线性化方法重写编程
% [version 9] 20220821 使用ode45算法重新改造机器人动力学计算方法,修正chipthickness与WorN函数，解决部分已知问题
% [version 10] 20220826 根据机器人铣削掉屑情况重写刀尖轨迹寄存器逻辑，实现四种掉屑情况的仿真；解决大量已知问题
% [version 11] 20221013 重新确认切削力方向，获取正确的铣削力和振动方向，修改随之产生的众多问题，启用数据过滤器
% [version 12] 20221108 加入铣削主轴，重新调整机器人几何结构，解决随之产生的众多问题
% [version 13] 20221202 参考骨铣削调整chip thickness计算策略
% [version 14] 20230119 根据Robot KUKA KR200机器人具体情况作修改

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%--------------Direction and Unit of physical quantities-----------------%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% *Unit of physical quantities
% unit of angle [rad];
% unit of length in cutting simulation [mm]
% unit of length in Robot simulation [m]
% unit conversion is important in this program. For some physical
% parameters, their [m] and [mm] version are presented respectively.

% *Reference frame 
% There are 3 reference frame in this robot, they are [machining plane 
% coordinate system] [TCP system ] and [inertia system]. These three
% reference coordinate system is illustrated here.
% *machining plane frame
% following case is its vertical view
%
%                          |x axis(feed direction)
%                          |
%                          |
%                          X———————— y axis
%                     z axis(inside)
% *TCP frame
% due to the import of milling spindle, orientation of TCP has changed.
%
%                          |x axis (pointing up)
%                          |
%                          |
%                          O———————— z axis(axis 6 = feed direction)
%                         /
%                        /
%                     y axis
% *inertia frame
% orientation of inertia frame depends on the world frame. relationship
% between orientation of TCP frame and inertia frame is
%                            P(inertia) = R*P(TCP)
%                         R = troty(-90) * trotx(180)
%
%                          |z axis (pointing up)
%                          | / y axis
%                          |/
%                          O———————— x axis(point to TCP)
%                      robot base

% *Reference cutting tool phasing 
% Definition of cutting tool phasing is determined by [reference] and 
% [positive direction]
% (1) positive directio: clockwise
% (2) reference: in [machining plane frame] -y direction
% calculated by function [atan2] in function [Cut_WorN] or by fun [mod]
%                                \
%                                 \
%                            theta \
%                          ________ \

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%-----------------------Robot Definition---------------------------------%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% * Robot import
% Info of robot is defined completely in document [Robot_Model], it is
% imported in workspace.
Robot_Model 

% * Initial robotic TCP define
% * robotic TCP is defined in [inertia frame]
% !Note that parameter [0.41] is distance between TCP and robotic flange.
TCP_x_m = Robot_TCP(1); % x coordinate of TCP in [inertia frame] [scalar,m]%%%%%%%%%%%【】
TCP_y_m = Robot_TCP(2); % y coordinate of TCP in [inertia frame]%%%%%%%%%%%%%%%%%%%%%%【】
TCP_z_m = Robot_TCP(3); % initial position of cutting tool [col,m,modify]%%%%%%%%%%%%%【】
TCP_x_mm = TCP_x_m*1000; % x coordinate of TCP in [inertia frame] [scalar,mm]
TCP_y_mm = TCP_y_m*1000; % y coordinate of TCP in [inertia frame] [scalar,mm]
TCP_z_mm = TCP_z_m*1000; % z coordinate of TCP in [inertia frame] [scalar,mm]
% * robotic TCP in [machining plane frame] can be calculated as below
Plane_x_m = TCP_x_m; 
% x direction is feed direction, it is the same as x in [inertia frame]
Plane_y_m = -TCP_y_m; 
% y direction points to left side of feed direction, it is opposite as y in
% [inertia frame]
Plane_xy_m = [Plane_x_m,Plane_y_m].'; % [col,m]
Plane_x_mm = TCP_x_mm; 
Plane_y_mm = -TCP_y_mm;
Plane_xy_mm = [Plane_x_mm,Plane_y_mm].';% [col,mm]

% * Feed speed
% During milling process,robot's trajectory is a uniform linear motion 
% along z-axis in [TCP frame] = x-axis in [inertia frame].
f_mm = 2.4; % [scalar,mm/s,modify]%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%【】
f_m = f_mm/1000; % feed speed [scalar,m/s]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%------------------Milling spindle Definition----------------------------%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Properties of cutting tool 
R = 10/2; % radius of cutting tool, [scalar,mm,modify]%%%%%%%%%%%%%%%%%%%%%【】
Helix = 0; % helix angle of cutting tool [scalar,deg,modify]%%%%%%%%%%%%%%%【】
Work = 0; % 0: half immersion (down milling)[scalar,0/1/2] 
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
N = Spindle_speed; % spindle speed of cutting tool, [scalar,rpm]
Ap = Axial_depth; % axial depth of cutting tool [scalar,mm]
W = N*2*pi/60; % angular velocity of cutting tool, [scalar,rad/s]
T = 2*pi/W; % Rotation period, [scalar,s]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%---------------------Simulation Preparation-----------------------------%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% * Simulation Parameters about Time and Sampling 
% Attention! [n] means [num = number], all variables having [n] is num of
% time steps.
sim_desire_time = 1; % desired simulation time [s]%%%%%%%%%%%%%%%%%%%%%%%%%【】
sim_n_dt_period = 120; % number of time steps in period; [scalar, modify]%%【】
sim_n_period = ceil(sim_desire_time/T); % number of total simulation period, [scalar,s,modify]
% * Derived Parameters
sim_T = T*sim_n_period; 
% Simulation time duration, [scalar,s]
[sim_dt,sim_n_dt_period] = Cut_Simudt(W,n_tip,sim_n_dt_period); 
% Length of every discrete time step, [scalar,s]
% number of time steps in a rotation period [scalar,s]
sim_n_dt_total = sim_n_dt_period*sim_n_period; 
% number of time steps in simulation duration [scalar]
sim_n_dt_work = sim_n_dt_period/n_tip; 
% number of time steps, in which a tip is in working, [scalar]
sim_n_start = 2; 
sim_n_dt_start = sim_n_dt_period * sim_n_start; 
% number of time steps required before simulation [scalar]

% * Simulation Parameters about Cutting tool
slice_n = 1;% num of slices in cutting tool [scalar,modify]
% * Derived Parameters
dz = Ap/slice_n; % height of slice [scalar,mm]
ds = dz/cosd(Helix); % length of cutting edge [scalar,mm]
CFC = Cut_ForceCalculator(Robot_R); % Cutting force coefficients[obj,modify]
% * Undercut flag determine [scalar 0/1]
UCflag_r1 = 0;
UCflag_r2 = 0;
UCflag_r3 = 0;
UCflag_r4 = 0;
% * Chip thickness filter
% in some cases, variation procedure of chip thickness may be [uniform 
% decrease]->[(sudden increase)]->[sharp decrease]. Among this process, 
% [sudden increase] may cause a lot of discrete points, which make the 
% figure chaotic. This [chip thickness filter] can exclude these points.
filter = 1; % chip thickness filter sign [scalar,0/1] default open

% * Simulation Parameters about Robot
% Simulation original point of trajectory 
TCP_desi_ori = Plane_xy_m; % [col,m] desired origin of trajectory
TCP_real_ori = Plane_xy_m; % [col,m] real origin of trajectory
% Model simplification option
sim_simply = 500; % calculate robotic state every [sim] points [scalar]%%%%【】
 
% * Info display
disp(strcat('machining condition: spindle speed n = ',num2str(N), ...
            ' [mm]; ','axial depth ap = ',num2str(Ap),' [mm]'))
disp('Influence of Robot Flexibility is considered during milling')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%---------------------Simulation Initialization ------------------------%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% * Registeer of TCP trajectory
% Real trajectory of TCP (cutting slice) is stored here.
Traj_TCP = zeros(3,sim_n_dt_total);

% * Register of ideal joint space trajectory and operation space trajectory
Traj_DesireOpera = zeros(sim_n_dt_total+1,3);
Traj_DesireJoint = zeros(sim_n_dt_total+1,6);

% * Register of updated feed
% Updated feed at each time step are stored here.
Feed_updated = zeros(2,sim_n_dt_total);

% * Register of Trajectory data in a period
% Trajectory data used to calculate chip thickness, store only effect
% working points in one period, size of register is 4脳[sim_n_dt_period]. 
% Row1:phase; Row2+3:position[x,y]; R4:work_sign
Traj_period_T1 = zeros(4,sim_n_dt_period);
Traj_period_T2 = zeros(4,sim_n_dt_period);
Traj_period_T3 = zeros(4,sim_n_dt_period);
Traj_period_T4 = zeros(4,sim_n_dt_period);

% * Register of Trajectory data for observation
% All data points in trajectory are stored here.
Traj_T1 = zeros(4,sim_n_dt_total);
Traj_T2 = zeros(4,sim_n_dt_total);
Traj_T3 = zeros(4,sim_n_dt_total);
Traj_T4 = zeros(4,sim_n_dt_total);

% * Register of Chip thicknesses of each teeth
% store chip thickness of Teeth i in the whole simulated process
% each register is a n-d vector.
Chip_thick_T1 = zeros(1,sim_n_dt_total); 
Chip_thick_T2 = zeros(1,sim_n_dt_total); 
Chip_thick_T3 = zeros(1,sim_n_dt_total); 
Chip_thick_T4 = zeros(1,sim_n_dt_total); 

% * Registers for cutting force of each teeth in moving frame TRA
% store cutting force of Teeth i in the whole simulated process, cutting
% force here include [tangential,radial and axial forces]
% each register is a 3*n matrix.
CutF_TRA_T1 = zeros(3,sim_n_dt_total); 
CutF_TRA_T2 = zeros(3,sim_n_dt_total); 
CutF_TRA_T3 = zeros(3,sim_n_dt_total); 
CutF_TRA_T4 = zeros(3,sim_n_dt_total); 

% * Registers for cutting force of each teeth in frame TCP
% store cutting force of Teeth i in the whole simulated process, cutting
% force here are projected in frame TCP
% each register is a 3*n matrix.
CutF_TCP_T1 = zeros(3,sim_n_dt_total); 
CutF_TCP_T2 = zeros(3,sim_n_dt_total); 
CutF_TCP_T3 = zeros(3,sim_n_dt_total); 
CutF_TCP_T4 = zeros(3,sim_n_dt_total); 

% Register for global cutting force
CutF_TCP = zeros(3,sim_n_dt_total);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%---------------Cutting Tool Slice initialization ----------------------%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Cutting tool slice definition
% input: [C_R] radius of cutting tool [mm]
%        [C_w] angular velocity of cutting force [rad/s]
%        [C_f] feed speed of cutting tool [mm/s]
%        [C_dt] length of time step [s]
%        [C_work_modal] current work modal [0/1/2],generally 0
%        [C_angle] initial phase of cutting tool [rad]
%        [C_centre] coordinate of cutter, [col,mm]
%        [n_TStep_inCycle] time steps'num in cycle [scalar]
CutTool_s = Cut_SliceClass(R,W,f_mm,sim_dt,Work,0,Plane_xy_mm,sim_n_dt_period);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ---------------Trajectory in joint space generation----------------- %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Trajectory of TCP in joint space is calculated by inverse kinematics, its
% corresponding function in RTB is [ikine]. vector calculation of this
% function has higher efficiency.
file_name = ['IdealTrajectory',num2str(N),'rpm_',num2str(sim_desire_time),num2str(sim_n_dt_period),'.mat'];
% ideal trajectory is stored in this name
if exist(file_name)
    % if ideal trajectory is calculated, it is then loaded
    load(file_name)
else
    % if there is no ideal trajectory, it will be calculated.
    % *totally [n+1] positions should be calculated for [n] time steps.
    % *Trajectory 1: trajectory in operation space and inertia frame [matrix,m]
    Traj_DesireOpera(:,1) = linspace(TCP_x_m, TCP_x_m+sim_n_dt_total*sim_dt*f_m, sim_n_dt_total+1)';
    Traj_DesireOpera(:,2) = ones(sim_n_dt_total+1,1) * TCP_y_m;
    Traj_DesireOpera(:,3) = ones(sim_n_dt_total+1,1) * TCP_z_m;
    % *Trajectory 2: homogenerous transformation matrix on trajectory [high order matrix]
    Traj_DesireHom = transl(Traj_DesireOpera);
    for t = 1:sim_n_dt_total+1
        Traj_DesireHom(:,:,t) = Traj_DesireHom(:,:,t) * Robot_R;
    end
    % *Trajectory 3: trajectory in joint space [row matrix, rad]
    Traj_DesireJoint = Robot_KUKA.ikine(Traj_DesireHom);
    save(file_name,"Traj_DesireJoint")
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%-------------------------Cyclic Simulation ----------------------------%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1:sim_n_dt_total
    % i: number of simulation time step
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %-------------------Feed in time step updating-----------------------%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % At predict instant (instant 2) of time step [i], cutting force is 
    % predicted and corrected by a [while] loop, a suitable [feed] in this 
    % time step at both x and y directions in [machining plane frame] are 
    % also calculated here.
    if i <= sim_n_dt_start 
        % if index <= simu_n_dt_start, the program is perparing for cutting,
        % robot during this process is seen as rigid.
        disp(strcat('Cycle Nr.',num2str(i),' in ',num2str(sim_n_dt_total),... 
             ' is over, but simulation hasn''t started yet. '))
    else
        % if index >= simu_n_dt_start+1, simulation starts, robot is began 
        % to be seen as rigid-flexible coupling structure.
        % ! In arbitrary time step, there are two notations:
        % current instant = beginning instant = instant 1
        % predict instant = final instant = instant 2.
        if i == sim_n_dt_start+1
            % Current desired and real TCP in [machining plane frame] [row,mm m]
            desi_1_TCP_mm = [CutTool_s.C_current_Centre_p].'; 
            desi_1_TCP_m = desi_1_TCP_mm/1000; 
            real_1_TCP_mm = [CutTool_s.C_current_Centre_p].'; 
            real_1_TCP_m = real_1_TCP_mm/1000;
            % Current desired generalized coordinate and velocity [col,rad rad/s]
            % their corresponding TCP coordinate is defined in [inertia frame]
            [desi_1_q,desi_1_qd] = Robot_LTraj(Robot_KUKA,[desi_1_TCP_m.*[1,-1],TCP_z_m],f_m,Robot_R);
            % Variation of generalized coordinate and velocity [col,rad rad/s]
            vq_1 = zeros(6,1);  
            vqd_1 = zeros(6,1); 
        else
            % Current desired and real TCP in [machining plane frame], [row,mm m]
            desi_1_TCP_mm = desi_2_TCP_mm; 
            desi_1_TCP_m = desi_2_TCP_m;
            real_1_TCP_mm = real_2_TCP_mm;
            real_1_TCP_m = real_2_TCP_m;
            % Current desired genralized coordinate and velocity [col,rad rad/s]
            desi_1_q = desi_2_q;
            % desi_1_qd = desi_2_qd;
            % Variation of generalized coordinate and velocity [col,rad rad/s]
            vq_1 = vq_2;
            vqd_1 = vqd_2;
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %-------Structural dynamics preparation-----%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        if i == sim_n_dt_start+1 || mod(i,sim_simply) == 0
            % Jacobian matrix [6*6 matrix]
            Jacb = Robot_KUKA.jacob0(desi_1_q);
            % Components in linearity equation of motion [6*6 matrix]
            M_zero = Robot_KUKA.inertia(desi_1_q.'); % Mass matrix [6*6]
            D_zero = Robot_KUKA.coriolis(desi_1_q.',desi_1_qd.'); % Christoffel term [6*6]
            C_zero = Robot_D; % Linearised Damping matrix [6*6]
            K_zero = Robot_K; % Linearised Stiffness matrix [6*6]
        end
        % Struct array used in function [ode45]
        Info.M = M_zero; % [struct]
        Info.C = C_zero + D_zero; % [struct]
        Info.K = K_zero; % [struct]

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %--------Structural dynamics solving-------%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        n_while = 0; % Number of Loop [scalar]
        feed_comparison = zeros(1,2); % Register of acceleration [row,mm]
        while 1
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %-----------Endless Loop Judgement------------%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % when num of Loop is too big,there must be a [endless loop]
            n_while = n_while + 1; % Number of Loop Updating
            if n_while > 5
                disp(err)
                msg = 'There may be a endless loop!';
                error(msg) % Endless loop occures, program stop!
            end 

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %------Predict cutting force calculation------%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Predict cutting force is exactly cutting force in instant 2.
            % These forces will be updated until uniform [feed] is obtained
            dw = CutTool_s.C_dw; % Phase variation in [sim_dt][scalar,rad]
            [C_T1,work_r1,UCflag,~,~] = Cut_ChipThickness(CutTool_s.T1_predict_tip_p, ... 
                                             Traj_period_T4(2,1:end), ...
                                             Traj_period_T4(3,1:end), ...
                                             Plane_xy_mm, ...
                                             CutTool_s.T1_working, ...
                                             dw);
            if UCflag == 1;UCflag_r1 = 1;end
            if filter == 1 && C_T1 > Chip_thick_T1(i) && Chip_thick_T1(i) ~= 0 && ...
                    C_T1/Chip_thick_T1(i) > 1.1
                C_T1 = 1e-20;
                % Exclude abnormally growing chip thickness after chip thickness decrease.
            elseif filter == 1 && C_T1 > Chip_thick_T1(i) && Chip_thick_T1(i) == 0 && ...
                    length(find(Chip_thick_T1(i-sim_n_dt_work:i)>1e-10))>1
                C_T1 = 1e-20;
                % Exclude abnormally growing chip thickness after 0 chip thickness.
            end
            Chip_thick_T1(i+1) = C_T1; % Chipthickness of teeth 1 [scalar mm]
            CutTool_s.T1_working = work_r1; % Updated work state of teeth 1 [scalar 0/1]
            [CutF_TRA_T1(:,i+1),CutF_TCP_T1(:,i+1)] = Cutting_Force(CFC, ... 
                                             Chip_thick_T1(i+1),dz,ds, ...
                                             CutTool_s.T1_predict_angle, ...
                                             CutTool_s.T1_working); % [col,N]
            
            [C_T2,work_r2,UCflag,~,~] = Cut_ChipThickness(CutTool_s.T2_predict_tip_p, ...
                                             Traj_period_T1(2,1:end), ...
                                             Traj_period_T1(3,1:end), ...
                                             Plane_xy_mm, ...
                                             CutTool_s.T2_working, ...
                                             dw); 
            if UCflag == 1;UCflag_r2 = 1;end
            if filter == 1 && C_T2 > Chip_thick_T2(i) && Chip_thick_T2(i) ~= 0 && ...
                C_T2/Chip_thick_T2(i) > 1.1
                C_T2 = 1e-10;
            elseif filter == 1 && C_T2 > Chip_thick_T2(i) && Chip_thick_T2(i) == 0 && ...
                    length(find(Chip_thick_T2(i-sim_n_dt_work:i)>1e-10))>1
                C_T2 = 1e-10;
            end
            Chip_thick_T2(i+1) = C_T2; % Chipthickness of teeth 2 [scalar mm]
            CutTool_s.T2_working = work_r2; % Updated work state of teeth 2 [scalar 0/1]
            [CutF_TRA_T2(:,i+1),CutF_TCP_T2(:,i+1)] = Cutting_Force(CFC, ...
                                             Chip_thick_T2(i+1),dz,ds, ...
                                             CutTool_s.T2_predict_angle, ...
                                             CutTool_s.T2_working); % [col,N]
            
            [C_T3,work_r3,UCflag,~,~] = Cut_ChipThickness(CutTool_s.T3_predict_tip_p, ...
                                             Traj_period_T2(2,1:end), ...
                                             Traj_period_T2(3,1:end), ...
                                             Plane_xy_mm, ...
                                             CutTool_s.T3_working, ...
                                             dw);
            if UCflag == 1;UCflag_r3 = 1;end
            if filter == 1 && C_T3 > Chip_thick_T3(i) && Chip_thick_T3(i) ~= 0 && ...
                C_T3/Chip_thick_T3(i) > 1.1
                C_T3 = 1e-10;
            elseif filter == 1 && C_T3 > Chip_thick_T3(i) && Chip_thick_T3(i) == 0 && ...
                    length(find(Chip_thick_T3(i-sim_n_dt_work:i)>1e-10))>1
                C_T3 = 1e-10;
            end
            Chip_thick_T3(i+1) = C_T3; % Chipthickness of teeth 3 [scalar mm]
            CutTool_s.T3_working = work_r3; % Updated work state of teeth 3 [scalar 0/1]
            [CutF_TRA_T3(:,i+1),CutF_TCP_T3(:,i+1)] = Cutting_Force(CFC, ...
                                             Chip_thick_T3(i+1),dz,ds, ...
                                             CutTool_s.T3_predict_angle, ...
                                             CutTool_s.T3_working); % [col,N]
            
            [C_T4,work_r4,UCflag,~,~] = Cut_ChipThickness(CutTool_s.T4_predict_tip_p, ...
                                             Traj_period_T3(2,1:end), ...
                                             Traj_period_T3(3,1:end), ...
                                             Plane_xy_mm, ...
                                             CutTool_s.T4_working, ...
                                             dw);
            if UCflag == 1;UCflag_r4 = 1;end
            if filter == 1 && C_T4 > Chip_thick_T4(i) && Chip_thick_T4(i) ~= 0 && ...
                C_T4/Chip_thick_T4(i) > 1.1
                C_T4 = 1e-10;
            elseif filter == 1 && C_T4 > Chip_thick_T4(i) && Chip_thick_T4(i) == 0 && ...
                    length(find(Chip_thick_T4(i-sim_n_dt_work:i)>1e-10))>1
                C_T4 = 1e-10;
            end
            Chip_thick_T4(i+1) = C_T4; % Chipthickness of teeth 4 [scalar mm]
            CutTool_s.T4_working = work_r4; % Updated work state of teeth 4 [scalar 0/1]
            [CutF_TRA_T4(:,i+1),CutF_TCP_T4(:,i+1)] = Cutting_Force(CFC, ...
                                             Chip_thick_T4(i+1),dz,ds, ...
                                             CutTool_s.T4_predict_angle, ...
                                             CutTool_s.T4_working); % [col,N]

            % Resultant cutting force in [inertia frame] [col,N]
            CutF_TCP_2 = CutF_TCP_T1(:,i+1) + CutF_TCP_T2(:,i+1) + ...
                         CutF_TCP_T3(:,i+1) + CutF_TCP_T4(:,i+1);
            % CutF_TCP_2 = [0 1 0].'*sin(i*0.1)*500; % Testing external cutting force
            CutF_TCP(:,i+1) = CutF_TCP_2; % [col,N]
            % disp(CutF_TCP_2) % Cutting force showcase

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %---------Structural dynamics solving---------%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Jacb_up = Jacb(1:3,:); 
            % Upper part of jacobian matrix [3*6 matrix]
            CutF_TCP_2 = Jacb_up.'*CutF_TCP_2; 
            % Resistant moments caused by cutting force in [inertia frame][col,Nm]
            Info.F = CutF_TCP_2; 
            % Struct array used in function [ode45][col,Nm]
            [vq_2,vqd_2] = Robot_Dynode45(i,sim_dt,vq_1,vqd_1,Info);
            % variation of genralized coordinate and velocity in instant 2 [col,rad rad/s]

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %-------Predict cutter position updating------%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if n_while == 1
                % Predict desired TCP in [machining plane frame][row,m mm]
                desi_2_TCP_m = desi_1_TCP_m + [f_m*sim_dt,0]; % desired feed!
                desi_2_TCP_mm = desi_2_TCP_m*1000;
                % Predict desired generalized coordinate and velocity [col,rad rad/s]
                % [desi_2_q,desi_2_qd] = Robot_LTraj(Robot_KUKA,desi_2_TCP_m,f_m);
                % Predict real generalized coordinate and velocity [col,rad rad/s]
                desi_2_q = Traj_DesireJoint(i+1,:).';
            end
            real_2_q = desi_2_q + vq_2; 
            % real_2_qd = desi_2_qd + vqd_2; 
            % Predict real TCP in [inertia frame][row,m mm]
            real_2_TCP_m = transl(Robot_KUKA.fkine(real_2_q));
            real_2_TCP_mm = real_2_TCP_m*1000;
            real_2_TCP_mm = real_2_TCP_mm(1:2);
            % Feed calculation
            % Feed is real TCP variation during time step [i], it is 
            % calculated in [inertia frame], but should be converted in 
            % [machining plane frame], their y axes are opposite!!!
            feed = real_2_TCP_mm - real_1_TCP_mm; % feed in [inertia frame][row,mm]
            feed = feed .* [1 -1]; % feed in [machining plane frame][row,mm]
            Feed_updated(:,i) = feed.'; % [col,mm]
            CutTool_s.Tool_pose_updating(feed(1:2))% [col,mm]

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %--------------Iteration Judgement------------%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Difference between neigbhoring feed [row,mm]
            err = norm(feed-feed_comparison);  
            % Recursion Judgement
            if err < 1e-5
                break
            else
                feed_comparison = feed;
                % disp(err) % variation of [feed] showcase
            end
        end
        % disp(strcat('n_cy = ',num2str(n_cy))) % iteration num showcase 

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %-----------Sampling info print------------%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        if mod(i,100) == 0
            disp(strcat("Cycle Nr.",num2str(i)," in ",num2str(sim_n_dt_total)," is over"))
        end
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %----------------Current real trajectory generation-----------------%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Real trajectory of each teeth in current instant (instant 1) of time
    % step [i] is captured in this part.
    % * Trajectory of teeth 1
    n_past = CutTool_s.S_num_past1; % num of past points [scalar]
    n_cout = i-n_past-sim_n_dt_work; % current point index [scalar]
    % [n_cout] is start-counting point, it comprises complex components
    % 1. [i] number of current data points
    % 2. [simu_n_dt_work] working time steps in a period, tooth i start  
    %    working after time step [simu_n_dt_work]*i. This conclusion 
    %    considers initial empty 1/4 period.
    % 3. [n_past] past points in previous periods
    % 4. [-1] to ensure that length of [Traj_period] is [sim_n_dt_period]?
    if n_cout > sim_n_dt_period
        n_cout = n_cout - sim_n_dt_period;
        % prevent that [n_cout] > [sim_n_dt_period],this phenomenon is also 
        % caused by [n_past] determination logic
    end
    if n_cout == 1
        Traj_period_T1 = Traj_period_T1(:,1:sim_n_dt_period);
        % At the beginning of a new cycle, trajectory register will be reset.
        UCflag_r1 = 0;
        % At the beginning of a new cycle, undercut flage will be reset!
    end
    if n_cout > 0
        if n_cout <= 5 && UCflag_r1 == 1
            % error('got it!')
            UCflag_r1 = 0;
            % To prevent that [UCflag] is set as 1 at beginning of
            % effective trajectory, the first 5 points is protection points.
            % This situation is caused by the logic of [n_past].
        end
        if UCflag_r1 == 0 || i <= sim_n_dt_start
            % In [perparation of cutting simulation], when [teeth is in
            % working] or [undercut doesn't occur], trajectory information 
            % is stored in trajector register.  
            Traj_period_T1(1,n_cout) = CutTool_s.T1_current_angle;
            Traj_period_T1(2,n_cout) = CutTool_s.T1_current_tip_p(1);
            Traj_period_T1(3,n_cout) = CutTool_s.T1_current_tip_p(2);
            Traj_period_T1(4,n_cout) = CutTool_s.T1_working;
            Traj_T1(1,i) = CutTool_s.T1_current_angle;
            Traj_T1(2,i) = CutTool_s.T1_current_tip_p(1);
            Traj_T1(3,i) = CutTool_s.T1_current_tip_p(2);
            Traj_T1(4,i) = CutTool_s.T1_working;
        end
    end

    % * Trajectory of teeth 2
    n_past = CutTool_s.S_num_past2;
    n_cout = i-n_past-sim_n_dt_work*2;
    if n_cout > sim_n_dt_period
        n_cout = n_cout - sim_n_dt_period;
    end
    if n_cout == 1
        Traj_period_T2 = Traj_period_T2(:,1:sim_n_dt_period);
        UCflag_r2 = 0;
    end
    if n_cout > 0
        if n_cout <= 5 && UCflag_r2 == 1
            UCflag_r2 = 0;
        end
        if UCflag_r2 == 0 || i <= sim_n_dt_start
            Traj_period_T2(1,n_cout) = CutTool_s.T2_current_angle;
            Traj_period_T2(2,n_cout) = CutTool_s.T2_current_tip_p(1);
            Traj_period_T2(3,n_cout) = CutTool_s.T2_current_tip_p(2);
            Traj_period_T2(4,n_cout) = CutTool_s.T2_working;
            Traj_T2(1,i) = CutTool_s.T2_current_angle;
            Traj_T2(2,i) = CutTool_s.T2_current_tip_p(1);
            Traj_T2(3,i) = CutTool_s.T2_current_tip_p(2);
            Traj_T2(4,i) = CutTool_s.T2_working;
        end
    end

    % * Trajectory of teeth 3
    n_past = CutTool_s.S_num_past3;
    n_cout = i-n_past-sim_n_dt_work*3;
    if n_cout > sim_n_dt_period
        n_cout = n_cout - sim_n_dt_period;
    end
    if n_cout == 1
        Traj_period_T3 = Traj_period_T3(:,1:sim_n_dt_period);
        UCflag_r3 = 0;
    end
    if n_cout > 0
        if n_cout <= 5 && UCflag_r3 == 1
            UCflag_r3 = 0;
        end
        if UCflag_r3 == 0 || i <= sim_n_dt_start
            Traj_period_T3(1,n_cout) = CutTool_s.T3_current_angle;
            Traj_period_T3(2,n_cout) = CutTool_s.T3_current_tip_p(1);
            Traj_period_T3(3,n_cout) = CutTool_s.T3_current_tip_p(2);
            Traj_period_T3(4,n_cout) = CutTool_s.T3_working;
            Traj_T3(1,i) = CutTool_s.T3_current_angle;
            Traj_T3(2,i) = CutTool_s.T3_current_tip_p(1);
            Traj_T3(3,i) = CutTool_s.T3_current_tip_p(2);
            Traj_T3(4,i) = CutTool_s.T3_working;
        end
    end

    % * Trajectory of teeth 4
    n_past = CutTool_s.S_num_past4;
    n_cout = i-n_past-sim_n_dt_work*4;
    if n_cout > sim_n_dt_period
        n_cout = n_cout - sim_n_dt_period;
    end
    if n_cout == 1 || n_cout > sim_n_dt_period
        Traj_period_T4 = Traj_period_T4(:,1:sim_n_dt_period);
        UCflag_r4 = 0;
    end
    if n_cout > 0
        if n_cout <= 5 && UCflag_r4 == 1
            UCflag_r4 = 0;
        end
        if  UCflag_r4 == 0 || i <= sim_n_dt_start
            Traj_period_T4(1,n_cout) = CutTool_s.T4_current_angle;
            Traj_period_T4(2,n_cout) = CutTool_s.T4_current_tip_p(1);
            Traj_period_T4(3,n_cout) = CutTool_s.T4_current_tip_p(2);
            Traj_period_T4(4,n_cout) = CutTool_s.T4_working;
            Traj_T4(1,i) = CutTool_s.T4_current_angle;
            Traj_T4(2,i) = CutTool_s.T4_current_tip_p(1);
            Traj_T4(3,i) = CutTool_s.T4_current_tip_p(2);
            Traj_T4(4,i) = CutTool_s.T4_working;
        end
    end
    % Current real TCP trajectory store
    Traj_TCP(:,i) = [CutTool_s.C_current_Centre_p/1000;Plane_x_m]; % [col,m]

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %---------------------------Info updating----------------------------%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Information updating for next instant
    % Predict parameters in instant 2 of time step i step becomes current
    % parameters in instant 1 of time step i+1; predict parameters in time
    % step i+1 will be updated.
    CutTool_s.Tool_info_updating;
end

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %----------------------Consequency Construction-------------------------%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Consequence print
n_rotation = CutTool_s.S_num_rotation1;
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp(strcat('% Totally:',num2str(n_rotation),' turns are simulated. %'))
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')

% Result storage
name = ['F:\Reuslt\SimRobotMill_KUKACoupleODE_Slice_',num2str(N),'rpm_',num2str(Ap),'mm.mat'];
save(name)

end