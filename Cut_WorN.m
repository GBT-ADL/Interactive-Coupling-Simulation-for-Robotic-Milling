function working_sign = Cut_WorN(work_modal,centre,tip_p)
% This function can determine whether the tooth is in work or not
% input:[work_modal] work modal of cutting tool, half immersion up/down
%       milling or full immersion [scalar]
%       [centre] centre of cutter [col,mm]
%       [tip_p] position of tool tip, judging the position of cutting tool [col,mm]
% output:[working_sign] a sign presenting current working situation [scalar,1/0]

% Attention! positive direction of angle = phase is clockwise sense (顺时针
% 方向), its definition is
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

% Input check
if ~iscolumn(centre)||~isscalar(work_modal)||~iscolumn(tip_p)
    error('Error: Wrong input type')
end

% immersion cases is also determined in this function
% 1. [angle] belongs to [0,pi], sin(angle) > 0:full immersion = half circle
% 2. [angle] belongs to [pi/2,pi], sin(angle) < 0 %% cos(angle) > 0:half
%    immersion = 0.25 circle + up milling 顺铣
%    Attention! in this case, origin of cutting occurs at 0.25T
% 3. [angle] belongs to [0,pi], sin(angle) > 0 %% cos(angle) < 0:half
%    immersion = 0.25 circle + down milling 逆铣

phase = atan2(tip_p(2)-centre(2),tip_p(1)-centre(1)); % phase of teeth

if work_modal == 0 % half immersion [down milling]
    % cutting tip is in working if and only if both phase and cut-in
    % position satisfy the requirment. Note the definitation of [phase]
    % Optional:[-1e-5 < 0]:防止由于时间步数整除导致的零星[零切削力]
    if phase > -pi/2 && phase <= pi/2+1e-5 && tip_p(2) >= -1e-5
        working_sign = 1;
    else
        working_sign = 0;
    end
elseif work_modal == 1 % half immersion [up milling]
    % Working condition of half immersion up milling should be designed
    % specificly.
    if sin(centre)>=0 && cos(centre)>=0
         working_sign = 1;
    else
         working_sign = 0;
    end
else % full immersion
    if sin(centre)>=0 
         working_sign = 1;
    else
         working_sign = 0;
    end
end
end