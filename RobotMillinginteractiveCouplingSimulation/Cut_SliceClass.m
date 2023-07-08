classdef Cut_SliceClass < handle
    % This class object can define a cutting tool slice.

    % * Introduction
    % This class is kernel of our algorithm, it defines a series of 
    % information about cutter and milling procedure in a time step.
    % These information can be divided into 2 parts in 2 instants:
    % (1) at the beginning of a time step, also called instant 1
    % (2) at the endding of time step, also called instant 2
    % their data are exchanged and calculated in endless cycle simulation.
    % * Units system
    % Units of length and angle here are [mm] and [rad], [m] won't be used
    % in milling simulation
    % ! Note that reference frame in this class is XY plane in [machining 
    % plane frame], it is also ZY plane in [TCP frame]. so that notation
    % of the imported parameter maybe [zy], but it is called here as [xy]
    % specific difference between their orientation is shown below.

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
    % milling situation is investigated in this coordinate system.
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
    
    properties
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %-------------- Default properties of cutting tool ---------------%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        C_R  % radius of cutting tool [scalar,mm]
        C_w  % angular velocity of cutting tool [scalar, rad/s]
        C_f  % feed speed of cutting tool [scalar, mm/s]
        C_n = 4 % number of teeth, to be developed [scalar]
        C_dt % length of time step [scalar,s]
        C_dw % angular variation in a time step [scalar,rad/s]
        C_feed_in_dt % theoretical feed in a time step [scalar,mm]
        C_work_modal % 0/1=half immersion down/up mill; 2=full immersion, 
        % to be developed [scalar,0/1/2]
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %--------- Current and Predict properties of cutting tool --------%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % (t) beginning of time step = [current] = [instant 1]
        % (t+dt) endding of this time step = [predict] = [instant 2]
        % There is recurssion relationship between info in [instant 1] and 
        % [instant 2] of neighboring time steps.
        C_current_Centre_p; % current position of tool centre [col,x,y mm]
        C_current_angle;    % current phase of cutter [scalar,rad]
        C_predict_Centre_p; % predict position of cutter centre [col,x,y,mm]
        C_predict_angle;    % predict phase of cutter [scalar,rad]
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %-------- Current and Predict properties of cutting Teeth 1 ------%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        T1_current_tip_p; % position of current Teeth 1 [col,x,y,mm]
        T1_current_angle; % angular position of current Teeth 1 [scalar,rad]
        T1_predict_tip_p; % position of predict Teeth 1 [col,x,y,mm]
        T1_predict_angle; % angular position of predict Teeth 1 [scalar,rad]
        T1_working;       % working sign, 1 present working [scalar 0/1]
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %-------- Current and Predict properties of cutting Teeth 2 ------%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % angular variation between every 2 tooth is [0.5*pi]. 
        T2_current_tip_p; % position of current Teeth 2 [col,x,y,mm]
        T2_current_angle; % angular position of current Teeth 2 [scalar,rad]
        T2_predict_tip_p; % position of predict Teeth 2 [col,x,y,mm]
        T2_predict_angle; % angular position of predict Teeth 2 [scalar,rad]
        T2_working;       % working sign, 1 present working [scalar 0/1]
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %-------- Current and Predict properties of cutting Teeth 3 ------%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        T3_current_tip_p; % position of current Teeth 3 [col,x,y,mm]
        T3_current_angle; % angular position of current Teeth 3 [scalar,rad]
        T3_predict_tip_p; % position of predict Teeth 3 [col,x,y,mm]
        T3_predict_angle; % angular position of predict Teeth 3 [scalar,rad]
        T3_working;       % working sign, 1 present working [scalar 0/1]
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %-------- Current and Predict properties of cutting Teeth 4 ------%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        T4_current_tip_p; % position of current Teeth 4 [col,x,y,mm]
        T4_current_angle; % angular position of current Teeth 4 [scalar,rad]
        T4_predict_tip_p; % position of predict Teeth 4 [col,x,y,mm]
        T4_predict_angle; % angular position of predict Teeth 4 [scalar,rad]
        T4_working;       % working sign, 1 present working [scalar 0/1]
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %------------------ Signals in Simulation  -----------------------%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        S_num_rotation1; % rotation number of teeth 1 [scalar]   
        S_num_rotation2; % rotation number of teeth 2 [scalar] 
        S_num_rotation3; % rotation number of teeth 3 [scalar] 
        S_num_rotation4; % rotation number of teeth 4 [scalar] 
        % work_sign of teeth i in previous time step
        S_T1_working; % work_sign of teeth 1 in previous time step[scalar]
        S_T2_working; % work_sign of teeth 2 in previous time step[scalar]
        S_T3_working; % work_sign of teeth 3 in previous time step[scalar]
        S_T4_working; % work_sign of teeth 4 in previous time step[scalar] 
        S_num_TimeStep_inCycle; % num of time step in a period [scalar]
        S_num_past1 % pasted data points on trajectory T1 [scalar]
        S_num_past2 % pasted data points on trajectory T2 [scalar]
        S_num_past3 % pasted data points on trajectory T3 [scalar]
        S_num_past4 % pasted data points on trajectory T4 [scalar]
        % * Approach of [S_num_rotation] updating
        % If work_sign of T1 in previous time step is 0 but in current 
        % time step is 1 (a "Raising edge" occurs), number of rotation +1.
        % * Usage of [S_num_past], data points are repeatedly stored in the
        % same register. To achieve this goal, it is necessary to determine
        % index of different data points. [S_num_past] is used for this
        % goal.
    end

    methods
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %-----------------------Construct Function------------------------%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj = Cut_SliceClass(C_R,C_w,C_f,C_dt,C_work_modal,C_angle,C_centre,n_TStep_inCycle)
            % Construct function, defining initial state of cutting tool
            % input: [C_R] radius of cutting tool [scalar,mm]
            %        [C_w] angular velocity of cutting force [scalar,rad/s]
            %        [C_f] feed speed of cutting tool [scalar,mm/s]
            %        [C_dt] length of time step [scalar,s]
            %        [C_work_modal] current work modal [scalar,0/1/2]
            %        [C_angle] initial phase of cutting tool [scalar,rad]
            %        [C_centre] coordinate of cutter, [col,mm]
            %        [n_TStep_inCycle] num of time steps in a period[scalar]
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %---------------------Input calibration-----------------------%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if ~isscalar(C_R)||~isscalar(C_w)||~isscalar(C_f)||~isscalar(C_dt)||...
               ~isscalar(C_work_modal)||~isscalar(C_angle)||~isscalar(n_TStep_inCycle)
                error('Error: Wrong input type')
            end
            if ~iscolumn(C_centre)
                error('Error: Wrong input type')
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %--------------Definition of default parameters---------------%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            obj.C_R = C_R;
            obj.C_w = C_w;
            obj.C_f = C_f;
            obj.C_dt = C_dt;
            obj.C_dw = obj.C_dt*obj.C_w;
            obj.C_feed_in_dt = obj.C_dt*obj.C_f;
            obj.C_work_modal = C_work_modal;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %--------------Definition of Cutter and tooth-----------------%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            
            % Definition of of cutting tool
            obj.C_current_Centre_p = C_centre;
            obj.C_current_angle = C_angle;
            obj.C_predict_Centre_p = obj.C_current_Centre_p + [obj.C_feed_in_dt,0]';
            obj.C_predict_angle = obj.C_current_angle + obj.C_dw;
            
            % Definition of cutting Teeth 1
            obj.T1_current_angle = obj.C_current_angle;
            obj.T1_current_tip_p = Cut_TipP(obj.C_current_Centre_p, ...
                                                obj.C_R,obj.T1_current_angle);
            obj.T1_predict_angle = obj.C_predict_angle;
            obj.T1_predict_tip_p = Cut_TipP(obj.C_predict_Centre_p, ...
                                                obj.C_R,obj.T1_predict_angle);
            if obj.T1_current_angle < 0
                obj.T1_working = 0;
            else
                obj.T1_working = Cut_WorN(obj.C_work_modal, ...
                                              obj.C_current_Centre_p, ...
                                              obj.T1_current_tip_p);
            end 
            
            % Definition of cutting Teeth 2
            obj.T2_current_angle = obj.T1_current_angle-0.5*pi;
            obj.T2_current_tip_p = Cut_TipP(obj.C_current_Centre_p,...
                                                obj.C_R,obj.T2_current_angle);
            obj.T2_predict_angle = obj.T1_predict_angle-0.5*pi;
            obj.T2_predict_tip_p = Cut_TipP(obj.C_predict_Centre_p, ...
                                                obj.C_R,obj.T2_predict_angle);
            obj.T2_working = 0;
            
            % Definition of cutting Teeth 3
            obj.T3_current_angle = obj.T2_current_angle-0.5*pi;
            obj.T3_current_tip_p = Cut_TipP(obj.C_current_Centre_p, ...
                                                obj.C_R,obj.T3_current_angle);
            obj.T3_predict_angle = obj.T2_predict_angle-0.5*pi;
            obj.T3_predict_tip_p = Cut_TipP(obj.C_predict_Centre_p, ...
                                                obj.C_R,obj.T3_predict_angle);
            obj.T3_working = 0;
            
            % Definition of cutting Teeth 4
            obj.T4_current_angle = obj.T3_current_angle-0.5*pi;
            obj.T4_current_tip_p = Cut_TipP(obj.C_current_Centre_p, ...
                                                obj.C_R,obj.T4_current_angle); 
            obj.T4_predict_angle = obj.T3_predict_angle-0.5*pi;
            obj.T4_predict_tip_p = Cut_TipP(obj.C_predict_Centre_p, ...
                                                obj.C_R,obj.T4_predict_angle);
            obj.T4_working = 0;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %-----------Definition of Signals in Simulation---------------%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if C_work_modal == 0
                           %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
                           %  halb immersion down milling % 
                           %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
            % principle to update the number of rotation is identification 
            % of "Raising edge". this strategy is totally right in 
            % [halb immersion up mill] and [full immersion], becasue tooth 
            % in these cases starts to work when cutting tool phase = 0.
            % But!! for [halb immersion down milling], tooth starts to work
            % when cutting tool phase = [pi/2], so that the number of
            % rotation is set as 1 once the tooth contact with workpiece
            % at the first time. To solve this problem, initial number of 
            % num_rotation is defined as [-1] in this case.
            obj.S_num_rotation1 = -1;
            obj.S_num_rotation2 = -1;
            obj.S_num_rotation3 = -1;
            obj.S_num_rotation4 = -1;
            obj.S_T1_working = 1;
            obj.S_T2_working = 1;
            obj.S_T3_working = 1;
            obj.S_T4_working = 1;
            obj.S_num_TimeStep_inCycle = n_TStep_inCycle;
            % past points of T1
            if obj.S_num_rotation1*obj.S_num_TimeStep_inCycle < 0
                obj.S_num_past1 = 0;
            else
                obj.S_num_past1 = obj.S_num_rotation1*obj.S_num_TimeStep_inCycle;
            end
            % past points of T2
            if obj.S_num_rotation2*obj.S_num_TimeStep_inCycle < 0
                obj.S_num_past2 = 0;
            else
                obj.S_num_past2 = obj.S_num_rotation2*obj.S_num_TimeStep_inCycle;
            end
            % past points of T3
            if obj.S_num_rotation3*obj.S_num_TimeStep_inCycle < 0
                obj.S_num_past3 = 0;
            else
                obj.S_num_past3 = obj.S_num_rotation3*obj.S_num_TimeStep_inCycle;
            end
            % past points of T4
            if obj.S_num_rotation4*obj.S_num_TimeStep_inCycle < 0
                obj.S_num_past4 = 0;
            else
                obj.S_num_past4 = obj.S_num_rotation4*obj.S_num_TimeStep_inCycle;
            end
            else 
                           %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
                           %     other wokring modals     % 
                           %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
            % except tooth 1, the other teeth have also the problem
            % mentioned above, their initial [S_num_rotation] are also [-1]
            obj.S_num_rotation1 = 0;
            obj.S_num_rotation2 = -1;
            obj.S_num_rotation3 = -1;
            obj.S_num_rotation4 = -1;
            obj.S_T1_working = 1;
            obj.S_T2_working = 1;
            obj.S_T3_working = 1;
            obj.S_T4_working = 1;
            obj.S_num_TimeStep_inCycle = n_TStep_inCycle;
            % past points of T1
            if obj.S_num_rotation1*obj.S_num_TimeStep_inCycle < 0
                obj.S_num_past1 = 0;
            else
                obj.S_num_past1 = obj.S_num_rotation1*obj.S_num_TimeStep_inCycle;
            end
            % past points of T2
            if obj.S_num_rotation2*obj.S_num_TimeStep_inCycle < 0
                obj.S_num_past2 = 0;
            else
                obj.S_num_past2 = obj.S_num_rotation2*obj.S_num_TimeStep_inCycle;
            end
            % past points of T3
            if obj.S_num_rotation3*obj.S_num_TimeStep_inCycle < 0
                obj.S_num_past3 = 0;
            else
                obj.S_num_past3 = obj.S_num_rotation3*obj.S_num_TimeStep_inCycle;
            end
            % past points of T4
            if obj.S_num_rotation4*obj.S_num_TimeStep_inCycle < 0
                obj.S_num_past4 = 0;
            else
                obj.S_num_past4 = obj.S_num_rotation4*obj.S_num_TimeStep_inCycle;
            end
            end 
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %-----------------------Tool pose updating------------------------%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function Tool_pose_updating(obj,feed)
        % Class approach: feed updating
        % * if the variation between predict and calculated feed in the same
        % time step is too big, feed needs to be updated. After updating,
        % current state has no change but predict positions is changed.
        % * spindle speed is unchanged by default!! so [predict_angle] has
        % also no difference.
        % input: [feed] new feed, [row,x,y,mm] 

        % Input calibration
        if ~isrow(feed)
            error('Error: Wrong input type')
        end
        % Info updating
            obj.C_predict_Centre_p = obj.C_current_Centre_p + [feed(1);feed(2)];
            obj.T1_predict_tip_p = Cut_TipP(obj.C_predict_Centre_p, ...
                                                obj.C_R,obj.T1_predict_angle);
            obj.T2_predict_tip_p = Cut_TipP(obj.C_predict_Centre_p, ...
                                                obj.C_R,obj.T2_predict_angle);
            obj.T3_predict_tip_p = Cut_TipP(obj.C_predict_Centre_p, ...
                                                obj.C_R,obj.T3_predict_angle);
            obj.T4_predict_tip_p = Cut_TipP(obj.C_predict_Centre_p, ...
                                                obj.C_R,obj.T4_predict_angle);
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %-----------------------Tool info updating------------------------%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function Tool_info_updating(obj)
        % Class approach: state updating
        % when next time step is comming, information of cutting tool
        % needs to be updated. In this function: Current values in new time
        % step is predict values in last time step, namely recursion.                              
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % -------------Current Parameter Assignment------------------%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Parameters of cutting tool 
            obj.C_current_angle = obj.C_current_angle + obj.C_dw;
            obj.C_current_Centre_p = obj.C_predict_Centre_p;

            % Parameters of Teeth 1
            obj.T1_current_tip_p = obj.T1_predict_tip_p;
            obj.T1_current_angle = obj.C_current_angle;
            % tooth won't work if their phase < 0
            if obj.T1_current_angle < 0
                obj.T1_working = 0;
            else
                obj.T1_working = Cut_WorN(obj.C_work_modal, ...
                                              obj.C_current_Centre_p, ...
                                              obj.T1_current_tip_p);
            end 

            % Parameters of Teeth 2
            obj.T2_current_tip_p = obj.T2_predict_tip_p;
            obj.T2_current_angle = obj.T1_current_angle-0.5*pi;
            % tooth won't work if their phase < 0
            if obj.T2_current_angle < 0
                obj.T2_working = 0;
            else
                obj.T2_working = Cut_WorN(obj.C_work_modal, ...
                                              obj.C_current_Centre_p, ...
                                              obj.T2_current_tip_p);
            end 

            % Parameters of Teeth 3
            obj.T3_current_tip_p = obj.T3_predict_tip_p;
            obj.T3_current_angle = obj.T2_current_angle-0.5*pi;
            % tooth won't work if their phase < 0
            if obj.T3_current_angle < 0
                obj.T3_working = 0;
            else
                obj.T3_working = Cut_WorN(obj.C_work_modal, ...
                                              obj.C_current_Centre_p, ...
                                              obj.T3_current_tip_p);
            end 

            % Parameters of Teeth 4
            obj.T4_current_tip_p = obj.T4_predict_tip_p;
            obj.T4_current_angle = obj.T3_current_angle-0.5*pi;
            % tooth won't work if their phase < 0
            if obj.T4_current_angle < 0
                obj.T4_working = 0;
            else
                obj.T4_working = Cut_WorN(obj.C_work_modal, ...
                                              obj.C_current_Centre_p, ...
                                              obj.T4_current_tip_p);
            end 

            % Signals of Simulation of T1
            if obj.T1_current_angle>0 && obj.T1_working>obj.S_T1_working
                obj.S_num_rotation1 = obj.S_num_rotation1 + 1;
            end
            obj.S_T1_working = obj.T1_working;
            if obj.S_num_rotation1*obj.S_num_TimeStep_inCycle < 0
                obj.S_num_past1 = 0;
            else
                obj.S_num_past1 = obj.S_num_rotation1*obj.S_num_TimeStep_inCycle;
            end

            % Signals of Simulation of T2
            if obj.T2_current_angle>0 && obj.T2_working>obj.S_T2_working
                obj.S_num_rotation2 = obj.S_num_rotation2 + 1;
            end
            obj.S_T2_working = obj.T2_working;
            if obj.S_num_rotation2*obj.S_num_TimeStep_inCycle < 0
                obj.S_num_past2 = 0;
            else
                obj.S_num_past2 = obj.S_num_rotation2*obj.S_num_TimeStep_inCycle;
            end

            % Signals of Simulation of T3
            if obj.T3_current_angle>0 && obj.T3_working>obj.S_T3_working
                obj.S_num_rotation3 = obj.S_num_rotation3 + 1;
            end
            obj.S_T3_working = obj.T3_working;
            if obj.S_num_rotation3*obj.S_num_TimeStep_inCycle < 0
                obj.S_num_past3 = 0;
            else
                obj.S_num_past3 = obj.S_num_rotation3*obj.S_num_TimeStep_inCycle;
            end

            % Signals of Simulation of T4
            if obj.T1_current_angle>0 && obj.T4_working>obj.S_T4_working
                obj.S_num_rotation4 = obj.S_num_rotation4 + 1;
            end
            obj.S_T4_working = obj.T4_working;
            if obj.S_num_rotation4*obj.S_num_TimeStep_inCycle < 0
                obj.S_num_past4 = 0;
            else
                obj.S_num_past4 = obj.S_num_rotation4*obj.S_num_TimeStep_inCycle;
            end
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % -----------------Predict values calculation-----------------%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Parameters of cutting tool 
            obj.C_predict_Centre_p = obj.C_current_Centre_p + [obj.C_feed_in_dt,0]';
            obj.C_predict_angle = obj.C_current_angle + obj.C_dw;
            % Parameters of Teeth 1
            obj.T1_predict_angle = obj.C_predict_angle;
            obj.T1_predict_tip_p = Cut_TipP(obj.C_predict_Centre_p, ...
                                                obj.C_R,obj.T1_predict_angle);
            % Parameters of Teeth 2
            obj.T2_predict_angle = obj.T1_predict_angle-0.5*pi;
            obj.T2_predict_tip_p = Cut_TipP(obj.C_predict_Centre_p, ...
                                                obj.C_R,obj.T2_predict_angle);
            % Parameters of Teeth 3
            obj.T3_predict_angle = obj.T2_predict_angle-0.5*pi;
            obj.T3_predict_tip_p = Cut_TipP(obj.C_predict_Centre_p, ...
                                                obj.C_R,obj.T3_predict_angle);
            % Parameters of Teeth 4
            obj.T4_predict_angle = obj.T3_predict_angle-0.5*pi;
            obj.T4_predict_tip_p = Cut_TipP(obj.C_predict_Centre_p, ...
                                                obj.C_R,obj.T4_predict_angle);
        end   
    end
end