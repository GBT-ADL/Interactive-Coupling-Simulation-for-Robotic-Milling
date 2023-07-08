%% Robot inertia parameters identification
% *Inertia information of robot KUKA KR500 R2830 MT is identified by SW and
% recorded here
% *6DOF robot is simplified as 3DOF model in this program, the real EE
% frame is exactly joint frame 3. To represent the flange more clear, a 
% joint frame 4 is also set. !! the final frame needs not be considered in
% dynamics analysis but should be included in statics analysis.

%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%--------------------Robot Information Definition------------------------%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% *D-H parameters
% * There are totally 4 frames locate on [base/joint 1],[joint2-3] and [EE]
% * L(1) stores D-H parameters between frame 0 and frame 1, namely from
%   [base/joint1] to [joint 2];
% * Base is neglected, so that the frame 1 locates on [joint 1] position 
%   of base is defined by [Robot.base()]
L(1) = Link('revolute','d', 0.479, 'a',0.500,'alpha',-pi/2, 'offset', 0); 
L(2) = Link('revolute','d', 0,     'a',1.300,'alpha', 0,    'offset',-pi/2);
L(3) = Link('revolute','d', 0,     'a',0.055,'alpha',-pi/2, 'offset', pi); 
L(4) = Link('revolute','d',-1.315, 'a',0,    'alpha', pi, 'offset', pi); 

% * Mass [m] [scalar,kg] COG [r] [row,m]
% reference of these physical parameters are corresponding joint frame.
L(1).m = 501; L(1).r = [-0.43,0.25,0.02];
L(2).m = 528; L(2).r = [-0.85,0,-0.31];
L(3).m = 398; L(3).r = [-0.02,0.02,-0.36];
L(4).m = 0; L(4).r = [0,0,0];

% * Centre inertia tensor [I] [kg.m^2]
% reference of these physical parameters are corresponding joint frame.
L(1).I = [25.58,-9.45,7.07;-9.45,53.74,-0.42;7.07,-0.42,56.22];
L(2).I = [14.38,0.32,-12.49;0.32,121.69,0.27;-12.49,0.27,122.67];
L(3).I = [90.75,-0.42,-2.98;-0.42,91.18,4.33;-2.98,4.33,7.71];
L(4).I = [0 0 0;0 0 0;0 0 0];

%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%-------------------------Robot Construction-----------------------------%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Robot_KUKA_test = SerialLink (L,'name','KUKA MIX'); % Serial Robot construction
Robot_KUKA_test.base = transl([0 0 0.566]); 

% Robot display                
Workspace = [-4,4,-4,4,-2,4];
q = [0, 30, 0 0]/180*pi;
Robot_KUKA_test.plot(q,'workspace',Workspace)  

% Rotation matrix between inertia frame and joint frame 4
R = t2r(Robot_KUKA_test.fkine(q));

% Robot mass matrix and Jacobian matrix
M = Robot_KUKA_test.inertia(q); % mass matrix 
J = Robot_KUKA_test.jacob0(q);
J_effect = J(1:3,1:3); % effect Jacobian matrix
disp(M)
disp(J_effect)
 



