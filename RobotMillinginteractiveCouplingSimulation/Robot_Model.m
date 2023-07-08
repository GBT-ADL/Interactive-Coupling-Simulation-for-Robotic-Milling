%% Rigid Robot Definition
% * All information about industrial robot KUKA KR500 is defined here, the
%   generated robot model is used to calculate robotic kinematics and rigid
%   dynamics.
% * unit of length and phase [m,rad]; [mm] isn't adopted in robotic model. 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Note! This program has been copied to:--------------------------------- %
%                   (1) Parameters identification                         %
%                   (2) Parameters optimization                           %
% After updating this program, these old files should be replaced by the  %
% new version                                                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [20221210]  robot model is updated w.r.t. robot KUKA KR210
% [20230129]  elastic parameters is updated after data processing
% [20230129]  Robotic TCP is updated 

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
% due to the import of milling spindle, orientation of TCP has changed
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

%{
% Version 1
% this robot model is built for [KUKA KR500]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%--------------------Robot Information Definition------------------------%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%% D-H parameters Definition [scalar,m rad] %%%%%%%%%%%%%%%%%
% * There are totally 6 links in robot KUKA KR500, robotic base is exclude
%   from this model,so that the frame 1 locates on [joint 1]
% * position of base is defined by [Robot.base()]
% * There are totally 7 frames locate on [base/joint 1],[joint2-6] and [TCP]
%   frame TCP represents state of both link 6 and milling spindle
% * L(1) stores D-H parameters between frame 0 and frame 1, namely from
%   [base/joint1] to [joint 2];
% * Data below is determined based on robot KUKA KR500
L(1) = Link('revolute','d', 0.479,       'a',0.500,'alpha',-pi/2, 'offset', 0); 
L(2) = Link('revolute','d', 0,           'a',1.300,'alpha', 0,    'offset',-pi/2);
L(3) = Link('revolute','d', 0,           'a',0.055,'alpha',-pi/2, 'offset', pi); 
L(4) = Link('revolute','d',-1.025,       'a',0,    'alpha', pi/2, 'offset', 0); 
L(5) = Link('revolute','d', 0,           'a',0,    'alpha',-pi/2, 'offset', 0); 
L(6) = Link('revolute','d',-(0.29+0.087),'a',0,    'alpha', pi,   'offset', pi); 
% !Note that parameter [0.87] is distance between spindle and motor side.

%%%%%%%%%%%%%%%%% Workspace Definition [scalar,rad] %%%%%%%%%%%%%%%%%%%%%%
% * L(1).qlim stores [min/max phase] of joint 1
% * joint phase interval [qlim] [rad]
% * Data below is determined based on robot KUKA KR500
L(1).qlim = [-185,185]/180*pi;  
L(2).qlim = [-130,20]/180*pi;
L(3).qlim = [-100,144]/180*pi;
L(4).qlim = [-350,350]/180*pi;
L(5).qlim = [-120,120]/180*pi;
L(6).qlim = [-350,350]/180*pi;

%%%%%%%%%% Inertia Parameters of Robot Definition [kg,m,kg.m^2] %%%%%%%%%%
% * Inertia parameters includes [mass], [centre of gravity = COG] wrt link
%   coordiante frame and [centre inertia tensor] define on body attached
%   frame locating on [COG]
% * Mass [m] [scalar,kg] COG [r] [row,m]
L(1).m = 558; L(1).r = [-0.409,  0.205,  0.035];
L(2).m = 485; L(2).r = [-0.847,  0.000, -0.310];
L(3).m = 311; L(3).r = [-0.033, -0.010,  0.043];
L(4).m = 86;  L(4).r = [ 0.000,  0.266, -0.011];
L(5).m = 48;  L(5).r = [ 0.000, -0.038, -0.052];
L(6).m = 7;   L(6).r = [ 0.000,  0.000, -0.027];
% * Centre inertia tensor [I] [kg.m^2]
L(1).I = [31.479 -11.062  9.269; -11.062 59.8450  1.849;  9.269  1.849 65.1610];
L(2).I = [13.322  11.567 -0.293;  11.567 113.722 -0.251; -0.293 -0.251 112.739];
L(3).I = [27.449 -0.1090 -0.301; -0.1090 25.0580  0.980; -0.301  0.980 10.5610];
L(4).I = [2.7830  0.0000  0.172;  0.0000 3.47100  0.000;  0.172  0.000 1.42300];
L(5).I = [0.7000  0.0000  0.000;  0.0000 0.68400  0.112;  0.000  0.112 0.43800];
L(6).I = [0.0180  0.0000  0.000;  0.0000 0.01800  0.000;  0.000  0.000 0.03400];

%%%%%%%%%%%%%% Elastic Parameters Definition [Nm*s/rad Nm/rad]%%%%%%%%%%%%
% * Joint stiffness and damping measured from EMA.
Robot_D = diag([1.9e3 1.4e3 0.6e3 1.9e3 1.4e3 1.4e3]); % [6*6,Nm*s/rad]
Robot_K = diag([2.3e6 8.6e6 2.8e6 3.0e6 0.2e6 0.9e6]); % [6*6,Nm/rad]

%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%-------------------------Robot Construction-----------------------------%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Robot_KUKA = SerialLink (L,'name','KUKA MIX'); % Serial Robot construction
Robot_KUKA.base = transl(0,0,0.566); % Base definition,height of base is 566 [mm]  
Robot_R = troty(-90)*trotx(180); % rotation matrix between inertia frame and TCP frame 

%}

%% 
% version 2 
% this robot model is built for [KUKA KR210], the objective robot in our
% project.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%--------------------Robot Information Definition------------------------%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%% D-H parameters Definition [scalar,m rad] %%%%%%%%%%%%%%%%%
% * There are totally 6 links in robot KUKA KR210, robotic base is exclude
%   from this model,so that the frame 1 locates on [joint 1]
% * position of base is defined by [Robot.base()]
% * There are totally 7 frames locate on [base/joint 1],[joint2-6] and [TCP]
%   frame TCP represents state of both link 6 and milling spindle
% * L(1) stores D-H parameters between frame 0 and frame 1, namely from
%   [base/joint1] to [joint 2];
% * Data below is determined based on robot KUKA KR210
L(1) = Link('revolute','d', 0.4494,      'a',0.350,'alpha',-pi/2, 'offset', 0); 
L(2) = Link('revolute','d', 0,           'a',1.150,'alpha', 0,    'offset',-pi/2);
L(3) = Link('revolute','d', 0,           'a',0.041,'alpha',-pi/2, 'offset', pi); 
L(4) = Link('revolute','d',-1.200,       'a',0,    'alpha', pi/2, 'offset', 0); 
L(5) = Link('revolute','d', 0,           'a',0,    'alpha',-pi/2, 'offset', 0); 
L(6) = Link('revolute','d',-0.420,       'a',0,    'alpha', pi,   'offset', pi); 
% !Note that parameter [0.42] is distance from frame 5 to motor side.

%%%%%%%%%%%%%%%%% Workspace Definition [scalar,rad] %%%%%%%%%%%%%%%%%%%%%%
% * L(1).qlim stores [min/max phase] of joint 1
% * joint phase interval [qlim] [rad]
% * Data below is determined based on robot KUKA KR500
L(1).qlim = [-185,185]/180*pi;  
L(2).qlim = [-130,20]/180*pi;
L(3).qlim = [-100,144]/180*pi;
L(4).qlim = [-350,350]/180*pi;
L(5).qlim = [-120,120]/180*pi;
L(6).qlim = [-350,350]/180*pi;

%%%%%%%%%% Inertia Parameters of Robot Definition [kg,m,kg.m^2] %%%%%%%%%%
% * Inertia parameters includes [mass], [centre of gravity = COG] wrt link
%   coordiante frame and [centre inertia tensor] define on body attached
%   frame locating on [COG]
% * Mass [m] [scalar,kg] COG [r] [row,m]
L(1).m = 259.85; L(1).r = [-0.36493,  0.22028,  0.00434];
L(2).m = 334.71; L(2).r = [-0.71793,  0.02787, -0.23573];
L(3).m = 217.46; L(3).r = [-0.02558,  0.02189, -0.31108];
L(4).m = 12.17;  L(4).r = [ 0.00000,  0.12343,  0.00135];
L(5).m = 29.53;  L(5).r = [ 0.00005, -0.02400, -0.04670];
L(6).m = 46.50;  L(6).r = [ 0.13734,  0.00000, -0.09334];
% * Centre inertia tensor [I] [kg.m^2]
L(1).I = [ 8.8821 -0.4015  1.0397; -0.4015 13.7205 -0.3202;  1.0397 -0.3202 16.2660];
L(2).I = [ 6.1923 -1.6859 -6.1251; -1.6859 55.8635  0.2354; -6.1251  0.2354 55.1699];
L(3).I = [25.4402 -0.0781  0.0171; -0.0781 25.9246  1.1022;  0.0171  1.1022  2.3116];
L(4).I = [ 0.1285  0.0000  0.0000;  0.0000  0.0298 -0.0028;  0.0000 -0.0028  0.1302];
L(5).I = [ 0.4021 -0.0001 -0.0001; -0.0001  0.2383 -0.0463; -0.0001 -0.0463  0.2879];
L(6).I = [ 0.2591  0.0000  0.0621;  0.0000  1.1944  0.0000;  0.0621  0.0000  1.1005];

%%%%%%%%%%%%%% Elastic Parameters Definition [Nm*s/rad Nm/rad]%%%%%%%%%%%%
% * Joint stiffness and damping measured from EMA
% these 2 matrices will be identified in elastic parameters identification
load('Robot_DK_28N6.mat')
Robot_D = D_result; % [6*6,Nm*s/rad]
Robot_K = K_result; % [6*6,Nm/rad]
% conclusion of elastic parameters matrix comes from [ParaIdentification_Final3_Stage4]

%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%-------------------------Robot Construction-----------------------------%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Robot_KUKA = SerialLink (L,'name','KUKA MIX'); % Serial Robot construction
Robot_KUKA.base = transl(0,0,0.2256); % Base definition,height of base is 225.6 [mm]  
Robot_R = troty(-90)*trotx(180); % rotation matrix between inertia frame and TCP frame 
Robot_TCP = transl(Robot_KUKA.fkine([-10.09 9.36 17.81 21.30 -28.85 -18.86]/180*pi));
% coordinate of robotic flange in [inertia frame], joint coordinate comes
% from teaching board! [row,m]

% Robot display                
% Workspace = [-4,4,-4,4,-1,4];
% Robot_KUKA.plot([0 0 0 0 0 0],'workspace',Workspace,'trail','b-')    
% Robot_KUKA.teach




