%% Result of CuttingTool Simulation Display

%% Parameter preparation
op = sim_n_start; % Observation origin [scalar]
ed = sim_n_period; % Observation end [scalar,modify]
disp_op = op*sim_n_dt_period; % Starting point of observation [scalar]
disp_ed = ed*sim_n_dt_period; % Endding point of observation [scalar]
time_op = disp_op * sim_dt; % Starting time [scalar,s]
time_ed = disp_ed * sim_dt; % Endding time [scalar,s]
disp_L = disp_ed-disp_op+1; % length of observation interval [scalar]

%%% Observation of Cutting force in frame TCP
% Data pre-treatment
% index of [0 0 0] in cutting force are stored in [i_index], only cutting
% force after [sim_n_dt_start] are required!.
L_CF = length(CutF_TCP);
% length of longer dimension of [CutF_TCP] [scalar]
j = 1;
% counter index [scalar]
for i = disp_op:L_CF
    if CutF_TCP(1,i) == 0 && CutF_TCP(2,i) == 0 && CutF_TCP(3,i) == 0 && i>sim_n_dt_start
        i_index(j) = i;
        j = j + 1;
    end
end

% Figure preparation
Units = 'centimeters';
Size1 = 18;
Size2 = 18;
FontName = 'Times New Roman';
F_op = -100; 
F_ed = 100; 

% Figure drawing
time = linspace(time_op,time_ed,disp_L);
fig1 = figure;
hold on
plot(time,CutF_TCP(1,disp_op:disp_ed),'-b','DisplayName','x axis','LineWidth',2) 
plot(time,CutF_TCP(2,disp_op:disp_ed),'-r','DisplayName','y axis','LineWidth',2)
plot(time,CutF_TCP(3,disp_op:disp_ed),'-g','DisplayName','z axis','LineWidth',2)
% Zero cutting force will be highlighted
if exist('i_index')
    L_Iindex = length(i_index);
    for j = 1:L_Iindex-1
        if i_index(j+1)-i_index(j) == 1
            x = [i_index(j),i_index(j+1),i_index(j+1),i_index(j)]*sim_dt;
            y = [F_ed,F_ed,F_op,F_op];
            fill(x,y,'r','FaceAlpha',0.3,'EdgeAlpha',0);
        end
    end
end

% Post Treatment
f1 = gcf;
f1.Units = 'centimeters';
f1.Name = 'Cutting force simulation';
% f1.Position = [5,5,20,15];
a1 = gca;
a1.FontSize = Size1;
a1.FontUnits = Units;
a1.FontName = FontName;
a1.XLabel.String = 'Time[s]';
a1.XLim = [time_op,time_ed]; % determine limite of x-axis
a1.XTick = linspace(time_op,time_ed,11); % determine ticket on x-axis
a1.YLabel.String = 'Cutting force [N]';
a1.YLim = [F_op,F_ed]; % determine limite of x-axis
a1.YTick = linspace(F_op,F_ed,6); % determine ticket on x-axis
a1.Title.String = 'Cutting force simulation';
% a1.Subtitle.String = 'Projected in frame TCP';

txt = {'\leftarrow Fx','\leftarrow Fy'};
tx = text(96,60,txt(1),'FontSize',Size2,'Color','b','FontName',FontName);
ty = text(65,420,txt(2),'FontSize',Size2,'Color','r','FontName',FontName);
tz = text(160,-100,{'\uparrow','Fz'},'FontSize',Size2,'Color','g','FontName',FontName);
% legend('Location','best')

%% Observation of Cutting force in frame TRA
fig2 = figure;
hold on
plot(CutF_TRA_T1(1,disp_op:end),'o','DisplayName','CutF TARt')
plot(CutF_TRA_T1(2,disp_op:end),'o','DisplayName','CutF TARr')
plot(CutF_TRA_T1(3,disp_op:end),'o','DisplayName','CutF TARa')
legend('Location','best')
title('Cutting force of tool tip 1 in frame TRA')

%% Observation of Trajectory of each slice
% Trajectory of slice 1 is indicated in this part
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
fig4 = figure;
hold on
for i = 1:slice_n
plot(Chip_thick_T1(i,disp_op:end),'o')
plot(Chip_thick_T2(i,disp_op:end),'o')
plot(Chip_thick_T3(i,disp_op:end),'o')
plot(Chip_thick_T4(i,disp_op:end),'o')
end
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
Traj_x = Traj_x(index); % Effect trajectory data on x direction [row,m]
Traj_y = Traj_y(index); % Effect trajectory data on y direction [row,m]
plot(Traj_y,Traj_x,'.-') 
title('Cutter trajectory')

fig5_2 = figure;
% Feed case showcase, y-axis trajectory of cutter [m]
plot(Traj_TCP(2,:),'.-','DisplayName','Simulation') 
Traj_y = Traj_TCP(2,:);
Traj_y = Traj_y(Traj_y~=0); % exclude 0 in trajectory
plot(Traj_y,'.-','DisplayName','Simulation') 
title('Cutter trajectory on y direction')

fig5_3 = figure;
% Feed case showcase, x-axis trajectory of cutter [m]
Traj_x = Traj_TCP(1,:);
Traj_x = Traj_x(Traj_x~=0); % exclude 0 in trajectory
Traj_x = Traj_x - linspace(1,length(Traj_x),length(Traj_x))*f_m*sim_dt;
Traj_x_absolute = Traj_x - TCP_x_m;
plot(Traj_x_absolute,'.-','DisplayName','Simulation') 
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