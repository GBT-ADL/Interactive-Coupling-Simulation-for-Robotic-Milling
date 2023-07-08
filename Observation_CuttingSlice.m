%% Result of CuttingSlice Simulation Display

%% Showcase interval definition
op = sim_n_start; % Observation origin [scalar]
ed = sim_n_period; % Observation end [scalar,modify]
disp_op = op*sim_n_dt_period+1; % Starting point of observation [scalar]
disp_ed = ed*sim_n_dt_period; % Endding point of observation [scalar]

%%% Observation of Cutting force in frame TCP
% Data pre-treatment
% index of [0 0 0] in cutting force are stored in [i_index], only cutting
% force after [sim_n_dt_start] are required!.
L_CF = length(CutF_TCP);
% length of longer dimension of [CutF_TCP] [scalar]
j = 1;
% counter index [scalar]
for i = disp_op:L_CF
    % Considering [Cut_T] filter, distinguish whether cutting force is 
    % small enough. Actually, data points processed by filter is so less
    % that they has almost no influence on showcasing of red shadow.
    condition1 = Observation_belongto(CutF_TCP(1,i),-1e-5,1e-5);
    condition2 = Observation_belongto(CutF_TCP(2,i),-1e-5,1e-5);
    condition3 = Observation_belongto(CutF_TCP(3,i),-1e-5,1e-5);
    if condition1 == 1 && condition2 == 1 && condition3 == 1 && i>sim_n_dt_start
        i_index(j) = i;
        j = j + 1;
    end
end
% limit define
F_op = min(CutF_TCP(:)); % Starting force [scalar,N]
F_ed = max(CutF_TCP(:)); % Endding force [scalar,N]

fig1 = figure;
hold on
plot(CutF_TCP(1,disp_op:disp_ed),'DisplayName','CutF TCPx','LineWidth',2) 
plot(CutF_TCP(2,disp_op:disp_ed),'DisplayName','CutF TCPy','LineWidth',2)
plot(CutF_TCP(3,disp_op:disp_ed),'DisplayName','CutF TCPz','LineWidth',2)
% Zero cutting force will be highlighted
if exist('i_index')
    L_Iindex = length(i_index);
    for j = 1:L_Iindex-1
        if i_index(j+1)-i_index(j) == 1
            x = [i_index(j),i_index(j+1),i_index(j+1),i_index(j)]-sim_n_dt_start;
            y = [F_ed,F_ed,F_op,F_op];
            fill(x,y,'r','FaceAlpha',0.3,'EdgeAlpha',0);
        end
    end
end
title('Cutting force of tool slice')

%% Observation of Cutting force in frame TRA
% Cutting force in frame TRA from teeth 1 are shown here.
fig2 = figure;
hold on
plot(CutF_TRA_T1(1,disp_op:disp_ed),'o','DisplayName','CutF TARt')
plot(CutF_TRA_T1(2,disp_op:disp_ed),'^','DisplayName','CutF TARr')
plot(CutF_TRA_T1(3,disp_op:disp_ed),'*','DisplayName','CutF TARa')
legend('Location','best')
title('Cutting force of tool tip 1 in frame TRA')

%% Observation of Trajectory of each slice
% Data pre-treatment: Data point [0,0] are exclude firstly.
L1 = length(Traj_T1);
L2 = length(Traj_T2);
L3 = length(Traj_T3);
L4 = length(Traj_T4);
for i = 1:L1
    if Traj_T1(3,i)==0 && Traj_T1(2,i)==0
        Traj_T1(3,i)=NaN;
        Traj_T1(2,i)=NaN;
    end
    if Traj_T2(3,i)==0 && Traj_T2(2,i)==0
        Traj_T2(3,i)=NaN;
        Traj_T2(2,i)=NaN;
    end
    if Traj_T3(3,i)==0 && Traj_T3(2,i)==0
        Traj_T3(3,i)=NaN;
        Traj_T3(2,i)=NaN;
    end
    if Traj_T4(3,i)==0 && Traj_T4(2,i)==0
        Traj_T4(3,i)=NaN;
        Traj_T4(2,i)=NaN;
    end
end
% Figure drawing: Trajectories of 4 tooth are shown here.
fig3 = figure;
hold on 
axis equal
plot(Traj_T1(3,:),Traj_T1(2,:),'.-','DisplayName','Traj T1');
plot(Traj_T2(3,:),Traj_T2(2,:),'.-','DisplayName','Traj T2');
plot(Traj_T3(3,:),Traj_T3(2,:),'.-','DisplayName','Traj T3');
plot(Traj_T4(3,:),Traj_T4(2,:),'.-','DisplayName','Traj T4');
legend('Location','best')
title('Tool tip trajectory')

%% Observation of chip thickness of each slice
% Chip thickness of 4 tooth are shown here.
fig4 = figure;
hold on
plot(Chip_thick_T1(disp_op:disp_ed),'o')
plot(Chip_thick_T2(disp_op:disp_ed),'o')
plot(Chip_thick_T3(disp_op:disp_ed),'o')
plot(Chip_thick_T4(disp_op:disp_ed),'o')
title('Chip thickness')

%% Observation of cutter trajectory
fig5_1 = figure;
% Complete Trajectory showcase,Trajectory of cutter at machining plane [m]
Traj_y = Traj_TCP(2,:); % trajectory data on y direction [row,m]
Traj_x = Traj_TCP(1,:); % trajectory data on x direction [row,m]
L_traj_obser = length(Traj_y); % length of trajectory register [scalar]
j = 1; % Register of cycle num [scalar]
for i = 1:L_traj_obser % Exclude data point (0,0) in trajectory
    if Traj_y(i) ~= 0 || Traj_x(i) ~=0
        index(j) = j;
        j = j+1;
    end
end
Traj_x = Traj_x(index); % Effect trajectory data on z direction [row,m]
Traj_y = Traj_y(index); % Effect trajectory data on y direction [row,m]
plot(Traj_y,Traj_x,'o') 
title('Cutter trajectory')

fig5_2 = figure;
% Feed case showcase, y-axis trajectory of cutter [m]
plot(Traj_TCP(2,:),'-o','DisplayName','Simulation') 
Traj_y = Traj_TCP(2,:);
Traj_y = Traj_y(Traj_y~=0); % exclude 0 in trajectory
plot(Traj_y,'-o','DisplayName','Simulation') 
title('Cutter trajectory on y direction')

fig5_3 = figure;
% Feed case showcase, x-axis trajectory of cutter [m]
Traj_x = Traj_TCP(1,:);
Traj_x = Traj_x(Traj_x~=0); % exclude 0 in trajectory
Traj_x = Traj_x - linspace(1,length(Traj_x),length(Traj_x))*f_m*sim_dt;
Traj_x_absolute = Traj_x - TCP_x_m;
plot(Traj_x - TCP_x_m,'-o','DisplayName','Simulation') 
title('Cutter trajectory on x direction')

%% Observation of Feed
% Feed in each time step can be indicated in this subsection
if exist('Feed_updated')
    Feed_x_updated = Feed_updated(1,:); % feed on x direction [col,mm]
    Feed_x_updated = Feed_x_updated(Feed_x_updated~=0) - f_mm*sim_dt; % effect feed on x direction [col,mm]
    fig6_1 = figure;
    plot(Feed_x_updated)
    title('Feed on x direction')

    Feed_y_updated = Feed_updated(2,:); % feed on y direction [col,mm]
    Feed_y_updated = Feed_y_updated(Feed_y_updated~=0); % effect feed on y direction [col,mm]
    fig6_2 = figure;
    plot(Feed_y_updated)
    title('Feed on y direction')
end