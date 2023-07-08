%% Result of CuttingSlice Simulation Display
% results of cuttings force is analysed in this program, SLD is also drawn
% here.
clc;clear;%close all
warning off

%% modify
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
peaks_value_threshold = 3.5; % peak's value should be bigger than [peaks_value_threshold] * [mean_value] [scalar]

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%--Data import and capture--%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
SS_cu = 5000; % current spindle speed [scalar rpm]
AP_cu = 1.5; % current axial depth of cut [scalar mm]
load('Sampling_info')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ---------------Data import-------------------%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
eval(strcat(' load(''SimRobotMill_KUKACoupleODE_Slice_',num2str(SS_cu), ...
    'rpm_',num2str(AP_cu),'mm.mat'')'))
SKip_points = sim_dt_min/sim_dt;
% to obtain uniform resolution, skip points between each 2 sampling points.
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
F_X = CutF_TCP(1,:); % cutting force on x direction [row,N]
F_Y = CutF_TCP(2,:); % cutting force on y direction [row,N]
F_Z = CutF_TCP(3,:); % cutting force on z direction [row,N]
CutF_TCP_global = sqrt(F_X.^2 + F_Y.^2 + F_Z.^2);
% CutF_TCP_global = CutF_TCP_global(CutF_TCP_global > 0);
% global cutting force on TCP [row,N]
MeanForce = mean(CutF_TCP_global);
% mean value of cutting force in x direction [scalar]
MaxForce = max(CutF_TCP_global);
% maximal value of cutting force in x direction [scalar]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% -----------------FFT implement------------------%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FFT is implemented for cutting forces in both x and y directions
% (corresponding to k = 1/2). the conclusions are stored respectively.
for k = 1:1
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
    fig = figure;
    hold on
    plot(f,P1)
    plot(f,peaks_value_threshold*P1_mean)

    % *peaks capturing and storage
    % two evaluation dimensions are set here to judge the peaks:
    % 1: gradients of peaks are big enough (bigger than
    %    [peaks_gradient_threshold] * [mean gradient])
    % 2: values of peaks are big enough (bigger than
    %    [peaks_value_threshold] * [mean_value])
    if k == 1
        Peaks_X_value = f(P1>P1_mean * peaks_value_threshold);
        Peaks_X_value = Peaks_X_value(Peaks_X_value > 5);
        Peaks_X_value = Peaks_X_value(Peaks_X_value < 1000);
        Peaks_X = Peaks_X_value;
        Peaks_X_value = Observation_VibFind(Peaks_X_value, N);
        NF_result_X = min(Peaks_X_value);
        WPD_result_X = Observation_EEntropy(Feed_updated(k,disp_op:SKip_points:SKip_points*sampling_points) - mean(Feed_updated(k,disp_op:SKip_points:SKip_points*sampling_points)),1);
        disp(WPD_result_X)
    elseif k == 2
        Peaks_Y_value = f(P1>P1_mean * peaks_value_threshold);
        Peaks_Y_value = Peaks_Y_value(Peaks_Y_value > 5);
        Peaks_Y_value = Peaks_Y_value(Peaks_Y_value < 1000);
        Peaks_Y = Peaks_Y_value;
        Peaks_Y_value = Observation_VibFind(Peaks_Y_value,N);
        NF_result_Y = min(Peaks_Y_value);
        WPD_result_Y = Observation_EEntropy(Feed_updated(k,disp_op:SKip_points:SKip_points*sampling_points) - mean(Feed_updated(k,disp_op:SKip_points:SKip_points*sampling_points)),1);     
        disp(WPD_result_Y)
    end
end





