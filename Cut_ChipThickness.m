function [C_t,work_result,undercut_flag,x_min,y_min] = Cut_ChipThickness(Ti_tip_p,traj_x,traj_y,ori,work,dw)
% This function can calculate [chip thickness] w.r.t. given trajectory and Tooth tip.
% * Thread of [chip thickness algorithm]
% After getting a set of trajectory points, this data points will be
% converted from cartesian coordinate frame to polar coordinate frame. 
% Polar coordinate is composed of "phase" and “distance”, these two 
% parameters can be calculated by function [atan2] and [norm]. Next, 
% these points will be exploited to fit a curve, if there is no repetition 
% in its x dimension, this is the reason why data points must be conversed 
% into polar coordinate system. To acquire more smoother curve, funtion 
% [pchip] (分段三次Hermite插值) can provide a suitable approach. After 
% generating some interrupt points around tool points, minimum distance 
% between it and the curve can be computed, compared and determined.

% input:[Ti_tip_p] position of tip of teeth Ti;[x,y,col,mm]
%       [last_trajectory] trajectory of tip of teeth Ti-1, it's x and y
%       coordinate are stored in two vector.[row,mm]
%       [ori] origin point of cutting tool trajectory [x,y,col,mm]
%       [work] work state of current tip [scalar]
%       [dw] phase variation in [sim_dt]

%       *Attention! Reference frame in operation space is EE frame = 
%       machining plane frame, its orientation is shown below.
%
%                          |x axis
%                          |
%                          |
%                          -———————— y axis
%       Reference: positive direction of x direction
%       postive direction: from [x] to [y] phase interval [0,pi]
%       negative direction: from [x] to [-y] phase interval [0,-pi] 
%       This function is achieved by function [atan2]

%       *Attention:definition of cutting tool phasing is shown below.
%       positive direction of Tool orientation angle = phase is clockwise 
%       sense (顺时针方向), its definition is shown below. This angle = 
%       phase can be calculated by function [atan2] in function [Cut_WorN] 
%       or by fun [mod]
%                                \
%                                 \
%                            theta \
%                          ________ \

%        *Attention: fun [atan2] is adopted here to calculate phasing of
%        tool tip, angles calculated by this algorithm from -pi(at -x axis)
%        to pi(at also -x axis) and increase along clockwise, its rule is
%        shown below
%                              y axis
%                                |pi/2
%                                |
%                        pi      |       0
%                        ————————-———————— x axis
%                        -pi     |
%                                |
%                                |-pi/2
%        when apply this approach in coordinate system in this program, its
%        phasing distribution is
%                              x axis
%                                |0
%                                |
%                        -pi/2   |     pi/2
%                        ————————-———————— y axis
%                                |
%                                |
%                             -pi|pi
%        function [atan2] is not as convenient as thicking, it involves
%        phasing reference change, this will cause confusion in phasing
%        calculation. In program further, it is replaced by fun [mod] by
%                    phasing = mod(current_angle,2pi)
%        However, why change a program that could run now? so that fun
%        [atan2] in old codes was accepted.

% output:[C_t] chip thickness, minimum distance between point and curve
%        [work_result] work state correction. When undercut occurs, work
%        state of current tip will be set as 0;
%        [undercut_flag] sign of undercut. When undercut occurs, undercut
%        sign will be set as 1. Different as [work result], it has no 
%        influence on work state. Once the [undercut_flag] is changed in a 
%        cycle, it won't be changed until the end of the cycle.
%        [undercut_flag] will be reset in new cycle.
%        [x_min,y_min] point in curve, distance between it and given point
%        are the shortest [scalar,mm]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%----------Data calibration ------------%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% * clear the warning
% warning('off'); 

% * Input calibration
if ~isrow(traj_x)||~isrow(traj_y)||~iscolumn(Ti_tip_p)||~isscalar(work)
    error('Error:wrong input type')
end

% * Work state determine
% Distinguish whether the cutting tool is working or not.if Cutting tool 
% isn't in working, it's chip thickness is 0
if work == 0 
    C_t = 0;
    work_result = 0;
    undercut_flag = 0;
    x_min = NaN;
    y_min = NaN;
    return
end  

% * Effect data capture
% Get effect parameters in [traj_x] and [traj_y], exclude useless point [0,0] 
L1 = length(traj_x); % lenght of [last_trajectory]
j = 1; % Index of effect parameters
for i = 1:L1
    if traj_x(i) ~= 0 || traj_y(i) ~= 0
        index(j) = i;
        j = j+1; % index increase
    end
end
% Distinguish whether the effect trajectory is empty or not
if ~exist('index','var')
    C_t = 0;
    work_result = 0;
    undercut_flag = 0;
    x_min = NaN;
    y_min = NaN;
    disp('ChipThickness:There is no effect data in last trajectory!')
    return
end    
% Generate effect trajectory
traj_x = traj_x(index);
traj_y = traj_y(index);
% Some data points are added at beginning and end of [traj_x] and [traj_y]
% to increase the robust of chip thickeness identification. Note 
% orientation of x and y axes.
% Head_add_y = linspace(traj_y(1)-5,traj_y(1),10);
% Head_add_x = ones(1,10)*traj_x(1);
% Tail_add_y = ones(1,10)*traj_y(end);
% Tail_add_x = linspace(traj_x(end),traj_x(end)-5,10);
traj_x = traj_x(1:end);% ,Tail_add_x
traj_y = traj_y(1:end);% ,Tail_add_y

% * Origin change
% origin point is converted to [0,0], so that data points in cartesian
% coordinate system can be converted into polar coordinate system simply.
traj_x = traj_x - ori(1);
traj_x(traj_x < -ori(1)) = 0; % [x dimension] of some points are negative
traj_y = traj_y - ori(2);
Ti_tip_p = Ti_tip_p - ori;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%----------Data Transformation ---------%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Data points in last trajectory is transformed from [Orthogonal coordinate
% system] to [Polar coordinate system]
L1 = length(traj_x); % lenght of last_trajectory
Theta_T = zeros(1,L1); % Register of phase [row,rad]
R_T = zeros(1,L1); % Register of radius [row,mm]
for i = 1:L1
Theta_T(i) = atan2(traj_y(i),traj_x(i)); % [row,rad]
% [atan2] function can calculate phase in the whole cycle, reference was changed.
R_T(i) = sqrt(traj_y(i)^2+traj_x(i)^2); % [row,mm]
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%------Candidate Segement determine-----%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Due to time difference between [tool tip] and [trajectory], phases of 
% [tool tip] and [goal point on trajectory] are not the same. To solve this
% question, some points on trajectory around tool tip are find as 
% [Candidate points]. The goal point who has the minimum distance with tool
% tip point will be found among them.

%%{ 
% old version chip thickness search 
% * Num of candidate points
n_candi = 100;
% * Candidate trajectory points generation
Theta_P = atan2(Ti_tip_p(2),Ti_tip_p(1)); % phase of Tool Tip [scalar,rad]
% * interval of candidate points determine, suitable for only [half immersion down milling]
i_candi = dw/200; % width of condidate interval 
Theta_ceil = Theta_P+i_candi; 
% upper limit of condidate interval, data in this side is close to old
% trajectory
if Theta_ceil > pi; Theta_ceil = pi; end
Theta_floor = Theta_P-i_candi/100;
% lower limit of condidate interval, data in this side is close to new
% trajectory
if Theta_floor < -pi; Theta_floor = -pi; end
Theta_candi = linspace(Theta_floor,Theta_ceil,n_candi); % condidate trajectory [row, rad]
Traj_T = pchip(Theta_T,R_T); % [struct] % fitting curve
R_candi = ppval(Traj_T,Theta_candi); % condidate trajectory [row,mm]
% * Candidate trajectory points transformation 
traj_y_candi = R_candi.*sin(Theta_candi); % [row,mm]
traj_x_candi = R_candi.*cos(Theta_candi); % [row,mm]
%%}

%{
% New version chip thickness search
% * Num of candidate points
n_candi = 1;
% * Candidate trajectory points generation
Theta_P = atan2(Ti_tip_p(2),Ti_tip_p(1)); % phase of Tool Tip [scalar,rad]
Traj_T = pchip(Theta_T,R_T); % [struct] % fitting curve
% R_candi = ppval(Traj_T,Theta_candi); % condidate trajectory [row,mm]
R_P = ppval(Traj_T,Theta_P); % condidate trajectory [row,mm]
% Major reversion! Refer to [bone milling], chip thickness calculation in
% geometric form was realized. In addition, strategy of chip thickness is
% no longer the minimum distance between previous trajectory and current
% tip, but locates on extension line of current tooth (当前未变形切屑厚度位
% 于当前刀齿的延长线上，与当前刀齿轨迹垂直). Data search is cancelled.
% * Candidate trajectory points transformation 
traj_y_candi = R_P.*sin(Theta_P); % [row,mm]
traj_x_candi = R_P.*cos(Theta_P); % [row,mm]
%}

% simulational result shows that difference between conclusions of Old and
% New version is too small to be observed.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%-------Chip thickness calculation------%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This subsection can select point in [condidata trajectory], distance 
% between it and point tool tip point is the minimum.
a_big_number = 5; % [scalar, modify]
distance_register = a_big_number; % register of candidate chip thickness. 
% Initial value here can be any big value, whose value must be bigger than 
% any possible chip thiknes.
for i = 1:n_candi % search in candidate curve
    x_distance_2 = (Ti_tip_p(1) - traj_x_candi(i))^2;
    y_distance_2 = (Ti_tip_p(2) - traj_y_candi(i))^2;
    distance_i = sqrt(x_distance_2 + y_distance_2);
    if distance_i < distance_register
        distance_register = distance_i;
        x_register = traj_x_candi(i);
        y_register = traj_y_candi(i);
    end
end
if distance_i > a_big_number
    % Error analysis
    % fig = figure;
    % hold on
    % axis equal
    % disp(strcat('its phase is:',num2str(Theta_P)))
    % plot(traj_y,traj_x,'o')
    % plot(traj_y_candi,traj_x_candi,'.')
    % plot(Ti_tip_p(2),Ti_tip_p(1),'o')
    % disp(['got it!'])
    % save('ztraj_x.mat','traj_x')
    % save('ztraj_y.mat','traj_y')
    % save('zR_T','R_T')
    % save('zTheta_T','Theta_T')
    % error('ChipThickness: There may be something wrong')%%%%%%%%%%%%%%
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%-------Chip thickness Calibration------%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% * Distinguish case of cutting
if distance_register >= a_big_number
    % if all points in last trajectory is [0 0], chip thickness is 
    % meaningless and chip thickness = 0;
    C_t = 0;
    work_result = 0;
    undercut_flag = 0;
    x_min = NaN;
    y_min = NaN;
elseif distance_register > 0 && norm(Ti_tip_p) < norm([x_register,y_register])
    % Under cut occurs when [ti_tip_p] is closer to origin compared with 
    % [x_min,y_min] in last trajectory. In this case, the tip hasn't cutted
    % the material, so chip thickness = 0;
    C_t = 0;
    work_result = 0;
    undercut_flag = 1;
    x_min = x_register;
    y_min = y_register;
    % disp('ChipThickness: Work of cutting teeth is completed in advance!')
elseif distance_register > 0 && Theta_P > pi/2
    % exclude abnormal data after working interval
    C_t = 0;
    work_result = 0;
    undercut_flag = 0;
    x_min = NaN;
    y_min = NaN;
    % disp('ChipThickness: Work of cutting teeth is completed delayed!')
elseif distance_register > 10
    % exclude abnormal data from working interval
        C_t = 0;
        work_result = 0;
        undercut_flag = 0;
        x_min = NaN;
        y_min = NaN;
    % disp('ChipThickness: Work of cutting teeth is completed delayed!')
else
    C_t = distance_register;
    work_result = work;
    undercut_flag = 0;
    x_min = x_register;
    y_min = y_register;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%---------Chip thickness check----------%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% upper-limite of chip thickness can be constrained and checked
% if C_t > 10 % C_t > 0.0061776 && C_t < 0.0061777
%     fig = figure;
%     hold on
%     axis equal
%     disp(strcat('Result of Chip thickness is:',num2str(C_t)))
%     disp(strcat('its phase is:',num2str(Theta_P)))
%     plot(traj_y,traj_x,'o')
%     plot(traj_y_candi,traj_x_candi,'.')
%     plot(Ti_tip_p(2),Ti_tip_p(1),'o')
%     plot(y_register,x_register,'^')
%     save('ztraj_x.mat','traj_x')
%     save('ztraj_y.mat','traj_y')
%     save('zR_T','R_T')
%     save('zTheta_T','Theta_T')
%     error('Check point reach!')
    % warning('Check point reach!')
    % input('Check point reach!')
% end
end