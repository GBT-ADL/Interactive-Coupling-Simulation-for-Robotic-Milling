%% Result of CuttingSlice Simulation Display
% results of cuttings force is analysed in this program, SLD is also drawn
% here.
clc;clear;% close all
warning off

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%
%--algorithm preparation--%
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% *Frequency spectral analysis parameters
NF_goal = [11,18,27]; % natural frequency of the first three modes [row, Hz]
NF_num = length(NF_goal); % num of desired natural frequencies [scalar]
Isolation_coefficient_threshold = 3; % Isolation coefficient, critical ratio of exciting frequency and natural frequency [scalar]
NF_threshold = max(NF_goal) * Isolation_coefficient_threshold; % threshold of natural frequency [scalar]
CutForce_mean_threshold = 80; % mean cutting force threshold [scalar]
CutForce_max_threshold = 100; % maximal cutting force threshold [scalar]
peaks_value_threshold = 3.8; % peak's value should be bigger than [peaks_value_threshold] * [mean_value] [scalar]

% *Simulation parameters_SS and AP
load(['Simulation_parameters.mat'])
Spindle_speed_op = 2000;
SS = Spindle_speed_op: Spindle_speed_interval: Spindle_speed_ed; % sequency of SS [row]
L_SS = length(SS); % num of Spindle speed [scalar]
AP = Axial_depth_op:axial_interval:Axial_depth_ed; % sequency of AP [row]
L_AP = length(AP); % num of axial depth of cut [scalar]

% *Data register
MeanForce = zeros(L_AP,L_SS);
% initial register of average cutting force
MaxForce = zeros(L_AP,L_SS);
% initial register of maxial cutting force
Peaks_X = cell(L_AP,L_SS);
% initial register of identified peaks on x direction
NF_pre_result_X = zeros(L_AP,L_SS);
% preliminary natural frequencies on x direction, fractional frequency
% multiplication is not considered
NF_result_X = zeros(L_AP,L_SS);
% final natural frequencies on x direction
Peaks_Y = cell(L_AP,L_SS);
% initial register of identified peaks on y direction
NF_pre_result_Y = zeros(L_AP,L_SS);
% preliminary natural frequencies on y direction, fractional frequency
% multiplication is not considered
NF_result_Y = zeros(L_AP,L_SS);
% final natural frequencies on y direction
WPD_result_X = zeros(L_AP,L_SS);
% result of wavelet package discomposition and energy entropy on x direction
WPD_result_Y = zeros(L_AP,L_SS);
% result of wavelet package discomposition and energy entropy on y direction


%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%--Data import and capture--%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for s = 1:L_SS
    SS_cu = SS(s); % current spindle speed [scalar rpm]
    for p = 1:L_AP
        AP_cu = AP(p);
        disp(strcat(num2str(SS_cu),'-',num2str(AP_cu)))
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % ---------------Data import-------------------%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        eval(strcat(' load(''SimRobotMill_KUKACoupleODE_Slice_',num2str(SS_cu), ...
            'rpm_',num2str(AP_cu),'mm.mat'')'))
        % * determined unified signal sampling resolution
        if s == 1 && p == 1
            sim_dt_min = sim_dt;    
            F_L = floor(length(Traj_TCP)/1000);
            sampling_points = 1000*F_L;
            % determine the biggest sim_dt, corresponding to the maximal
            % sampling resolution
        end
        SKip_points = sim_dt_min/sim_dt;
        save('Sampling_info','sim_dt_min','sampling_points')
        % to obtain uniform resolution, skip points between each 2 sampling
        % points.
        
        TPF = N/60 * 4; % Tool passing frequency
        % *simulation time
        op = sim_n_start; % Observation origin [scalar]
        ed = sim_n_period; % Observation end [scalar,modify]
        disp_op = op*sim_n_dt_period+1; % Starting point of observation [scalar]
        disp_ed = ed*sim_n_dt_period; % Endding point of observation [scalar]
        time_op = disp_op * sim_dt; % Starting time [scalar,s]
        time_ed = disp_ed * sim_dt; % Endding time [scalar,s]
        disp_L = disp_ed-disp_op+1; % length of observation interval [scalar]
        % *FFT settings
        FFT_Fs = 1/sim_dt; % sampling frequency [scalar,Hz]
        FFT_T = sim_dt; % sampling period [scalar,s]
        FFT_L = (time_ed - time_op) / sim_dt; % num of sampling points [scalar]
        FFT_t = (0:FFT_L) * FFT_T; % sampling time sequence

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % ---Cutting force pre-treatment and storage---%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        F_X = CutF_TCP(1,disp_op:disp_ed); % cutting force on x direction [row,N]
        F_Y = CutF_TCP(2,disp_op:disp_ed); % cutting force on y direction [row,N]
        F_Z = CutF_TCP(3,disp_op:disp_ed); % cutting force on z direction [row,N]
        CutF_TCP_global = sqrt(F_X.^2 + F_Y.^2 + F_Z.^2); 
        CutF_TCP_global = CutF_TCP_global(CutF_TCP_global > 0);
        % global cutting force on TCP [row,N]
        MeanForce(p,s) = mean(CutF_TCP_global); 
        % mean value of cutting force in x direction [scalar]
        MaxForce(p,s) = max(CutF_TCP_global); 
        % maximal value of cutting force in x direction [scalar] 
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % -----------------FFT implement------------------%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % FFT is implemented for cutting forces in both x and y directions
        % (corresponding to k = 1/2). the conclusions are stored respectively.
        for k = 1:2
            % *FFT drawing
            % both-side frequency spectrum are calculated firstly, the 
            % single-side frequency spectrun is then captured and showcased.
            Feed_cut = fft(Feed_updated(k,disp_op:disp_ed) - mean(Feed_updated(k,disp_op:disp_ed)));
            % *[Feed_updated] is updated "feed" in every sampling instant, it is
            % exactly "vibration" or "deviation" between ideal trajectory and real
            % trajectory. it is realy the "vibration signal"
            % * to calculate its FFT, mean value of [Feed_updated] is subtracted
            % from itself to exclude the "DC component". in vibration signal, only
            % "AC component" concludes effect information, "DC components" will
            % lead to peaks at 0 Hz of frequency spectrum, which influences the
            % result of energy entropy.
            P2 = abs(Feed_cut/FFT_L);
            P1 = P2(1:FFT_L/2 + 1);
            P1(2:end-1) = 2*P1(2:end-1);
            f = FFT_Fs * (0:(FFT_L/2))/FFT_L;
            
            % *peaks capture by value
            P1_mean = Observation_LocalMean(P1);
            % fig = figure;
            % hold on
            % plot(f,P1)
            % plot(f,ones(size(P1))*peaks_value_threshold*P1_mean)
            
            % *peaks capturing and storage
            %  1. values of peaks should be big enough (bigger than 
            %  [peaks_value_threshold] * [mean_value]) average here are not
            %  general mean value but 'local mean value', more about it is
            %  in function [observation_localmean]
            %  2. to prevent 'positive feedback vibration', a checker is
            %  set in function [observation_Vibfind]
            if k == 1
                FFvalue = Observation_FPeakFind(P1,f,TPF);
                DFvalue = Observation_FPeakFind(P1,f,TPF/2);
                DFvalue2 = Observation_FPeakFind(P1,f,TPF/3);
                NF_result_X(p,s) = max(DFvalue/FFvalue,DFvalue2/FFvalue);
                WPD_result_X(p,s) = Observation_EEntropy(Feed_updated(k,disp_op:SKip_points:SKip_points*sampling_points) - mean(Feed_updated(k,disp_op:SKip_points:SKip_points*sampling_points)),0);
            elseif k == 2   
                FFvalue = Observation_FPeakFind(P1,f,TPF);
                DFvalue = Observation_FPeakFind(P1,f,TPF/2);
                DFvalue2 = Observation_FPeakFind(P1,f,TPF/3);
                NF_result_Y(p,s) = max(DFvalue/FFvalue,DFvalue2/FFvalue);
                WPD_result_Y(p,s) = Observation_EEntropy(Feed_updated(k,disp_op:SKip_points:SKip_points*sampling_points) - mean(Feed_updated(k,disp_op:SKip_points:SKip_points*sampling_points)),0);
            end            
        end 
    end
end

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%-------Data showcase-------%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% meshgrid generation
figure_x = SS;
figure_y = AP;
[figure_X, figure_Y] = meshgrid(figure_x,figure_y);

% frequency division on x direction
fig1 = figure;
hold on
contourf(figure_X,figure_Y,NF_result_X);
contour(figure_X,figure_Y,NF_result_X,[0.5,0.5], ...
        'r','LineWidth',5);
colorbar
% frequency division on y direction
fig2 = figure;
hold on
contourf(figure_X,figure_Y,NF_result_Y);
contour(figure_X,figure_Y,NF_result_Y,[0.5,0.5], ...
        'r','LineWidth',5);
colorbar

% mean cutting force
fig5 = figure;
hold on
contourf(figure_X,figure_Y,MeanForce);
contour(figure_X,figure_Y,MeanForce, ...
        [CutForce_mean_threshold, CutForce_mean_threshold],'r','LineWidth',5);
colorbar
% maximal cutting force
fig6 = figure;
hold on
contourf(figure_X,figure_Y,MaxForce);
contour(figure_X,figure_Y,MaxForce, ...
        [CutForce_max_threshold, CutForce_max_threshold],'r','LineWidth',5);
colorbar

% energy entropy on x direction
fig7 = figure;
hold on
contourf(figure_X,figure_Y,WPD_result_X);
contour(figure_X,figure_Y,WPD_result_X,[0.3 0.3],'-r','LineWidth',5);
colorbar
% energy entropy on y direction
fig8 = figure;
hold on
contourf(figure_X,figure_Y,WPD_result_Y);
contour(figure_X,figure_Y,WPD_result_Y,[0.4 0.4],'-r','LineWidth',5);
colorbar




