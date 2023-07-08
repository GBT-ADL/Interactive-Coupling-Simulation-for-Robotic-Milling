classdef Cut_ForceCalculator < handle
    % This Class can calculate cutting force of each teeth on Cutting tool slice
    % *Note that! reference frame of cutting force in operation frame is
    % inertia frame but not machining plane frame

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

    % [20230119] cutting force coefficients are repleased by identified
    % parameters in experiments.
    % [20230130] cutting force coefficients are replaced again after a new
    % round of data processing
    
    properties
        Kc % Cutting force coefficients at T/R/A directions [N/mm^2] = 10e6*[N/m^2]
        Ke % Cutting edge coefficients at T/R/A direction [N/mm] -> 10e3*[N/m]
        R  % rotation matrix of robot from inertia frame to TCP frame [3*3matrix]
    end

    methods
        function obj = Cut_ForceCalculator(R,Kc,Ke)
            % Define cutting force / Cutting edge coefficients
            % input [Kc] Cutting force coefficients [row,N/m^2 = Pa]
            %       [Ke] Cutting edge coefficients,[row,N/m]
            % Default parameters 
            if nargin < 1
                R = [0  0  1;
                     0 -1  0;
                     1  0  0];
                % this matrix depends on form of cutting force in TCP frame below.
                % it corresponds to milling on top surface of workspiece.
                Kc = [1197.7,3857.5,-518.0]; 
                % cutting force coefficients [N/mm^2] = 10^-6*[N/m^2]
                Ke = [-1.10,-20.4,2.4] ; 
                % edge cutting coefficients [N/mm] -> 10^-3*[N/m]
                % Available Cutting coefficients
                % Kc = [2860.5,1046.2,1088.2]; % [N/mm^2] = 10^-6*[N/m^2]
                % Ke = [24,43,0] ; % [N/mm] -> 10^-3*[N/m]
                % Kc = [3361.3,4748.6,7296.3]; 
                % Ke = [-11.1,-27.8,-22.3] ; 
                % Kc = [1552.7,401.8,1422.8]; 
                % Ke = [0,0,0] ; 
                % Kc = [1197.7,3857.5,-518.0]; 
                % Ke = [-1.10,-20.4,2.4] ; 

                % 较成熟的两种铣削力系数
                % Group 1：
                % Kc = [1197.7,3857.5,-518.0]; 
                % Ke = [-1.10,-20.4,2.4] ; 
                % Group 2：
                % Kc = [3361.3,4748.6,1575.7]; 
                % Ke = [-11.1,-27.8,-22.3] ; 
                % [1575.7] comes from Z direction of concise model para!
            elseif nargin <2
                Kc = [1197.7,3857.5,-518.0]; 
                % cutting force coefficients [N/mm^2] = 10^-6*[N/m^2]
                Ke = [-1.10,-20.4,2.4] ; 
                % edge cutting coefficients [N/mm] -> 10^-3*[N/m]
                % Available Cutting coefficients
            end
            % input check
            if ~isrow(Kc)||~isrow(Ke),error('Error: Wrong input type');end
            % function body
            obj.R = R(1:3,1:3);
            obj.Kc = Kc.';
            obj.Ke = Ke.';
        end

        function [CutF_TRA,CutF_TCP] = Cutting_Force(obj,dh,db,ds,W,Work)
            % Cutting force in TRA and TCP frame calculation
            % input: [dh] chip thickness [scalar,mm]
            %        [db = dz] height of slice [scalar,mm]
            %        [ds] length of cutting edge [scalar,mm]
            %        [W] phase of current tooth [scalar,rad]
            %        [Work] used to distinguish the tooth is in work or not [bool, 1/0]
            % output: [cutF_TCP] cutting forces in frame TCP, a column vector [col,N]
            %         [cutF_TRA] cutting forces in moving frame TRA, a column vector [col,N]

            % Input Check
            if ~isscalar(dh)||~isscalar(db)||~isscalar(ds)||~isscalar(W)||~isscalar(Work)
                error('Error: Wrong input type')
            end

            % Cutting force projected in TRA frame 
            % *Note that directions of cutting forces
            % cutting force and supporting force applided on EE are a
            % couple of interaction force (相互作用力), they have the same 
            % magnitudes but opposite directions. the desired "cutting 
            % force" here is exactly supporting force provided by robot's
            % joints, so that directions of cutting force TRA are:
            % (1) t: along the cutting speed (to workspiece)
            % (2) r: along the local normal of the tool envelop (to outside)
            % (3) a: along the third direction generating an orthogonal
            % frame (perpendicular machining face and upward)
            % *Note that the reference frame of cutting force
            % reference frame of cutting force is [inertia frame] but not
            % machining plane frame.
            % to sum up, [directions] define values of cutting force, 
            % [reference frame] defines symbol of cutting force.
            CutF_m = obj.Kc*(dh*db) + obj.Ke*(ds); % [col,N]
            CutF_TRA = CutF_m * Work; % [col,N]
            CutF_mt = CutF_m(1);
            CutF_mr = CutF_m(2);
            CutF_ma = CutF_m(3);

            % Cutting force projected in inertia frame directly
            % reference coordinate system of cutting force is [inertia frame]
            % CutF_TCP = [(CutF_mt*cos(pi-W)-CutF_mr*sin(pi-W)); % x direction                                      % x direction
            %             (CutF_mt*sin(pi-W)+CutF_mr*cos(pi-W)); % y direction
            %             CutF_ma] * Work;                       % z direction

            % Cutting force projecgted in TCP frame
            % Note that this function depends on robotic configuration.
            CutF_TCP = [ CutF_ma;                                       % x direction
                        -(CutF_mt*sin(pi-W)+CutF_mr*cos(pi-W));         % y direction
                         (CutF_mt*cos(pi-W)-CutF_mr*sin(pi-W))] * Work; % z direction
            % Cutting force projected in TCP frame, it should be processed
            % by [rotation matrix R] to be converted into inertia frame.
            % frame conversion in operation space is implemented by
            % rotation matrix [R], this matrix is defined in [model_KUKA]
            CutF_TCP = obj.R * CutF_TCP;

            % CutF_TCP is [col,N]
            % if teeth don't work currently, its cutting force is set as 0 
            % by its 'Work_sign'
        end
    end
end