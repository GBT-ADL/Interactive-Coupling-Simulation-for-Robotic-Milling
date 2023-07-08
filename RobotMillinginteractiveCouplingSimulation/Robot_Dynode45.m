function [vq_result,vqd_result] = Robot_Dynode45(a,simu_dt,vq,vqd,Info)
% This function is used solve robotic structural dynamic equation by function [ode45]
% input:[Info] struct comprising [mass matrix 6*6][christoffel term 6*6]
%              [damping matrix6*6], [stiffness matrix6*6] and [external force, row N]. 
%              Theyare used to generate ordinary differential equation
%       [vq,vqd] deformation of generalized coordinate and its derivative 
%                at current instant, used as reference[row,rad,rad/s]
%       [simu_dt] length of time step [scalar,s]
%       [a = i] index of current time step [scalar]
% output:[vq_result] deformation of generalized coordinate, equation solution [col,rad]
%        [vqd_result] deformation of generalized velocity, equation solution [col rad/s]

% Data calibration
vq = Robot_V2Col(vq);
vqd = Robot_V2Col(vqd);
if ~isscalar(a)||~isscalar(simu_dt)||~iscolumn(vq)||~iscolumn(vqd)
    error('There may be something wrong!')
end

% Robotic structural dynamic equation solve
tspan = [(a-1)*simu_dt,a*simu_dt]; % time interval in time step [row,s]
vQ_1 = [vq;vqd]; % current state variables, as reference [col 12d,rad rad/s]
opts = odeset('Mass',@(t,Q) mass(t,Q,Info),'InitialStep',simu_dt);
[~,vQ] = ode45(@(t,Q) robotdynamic(t,Q,Info),tspan,vQ_1,opts);
vq_result = vQ(end,1:6)'; % Attention! joint deformation!
vqd_result = vQ(end,7:12)';

% Function used in [ode45]
% Mass matrix: coefficient of ordinary derivative item 
function MM = mass(t,Q,Info)
C = Info.C;
M = Info.M;
S = length(C);
Z = zeros(S,S);
MM = [C,M;
      M,Z];
end

% Ordinary differential equation  
function dydt = robotdynamic(t,Q,Info)
F = Info.F;
K = Info.K;
M = Info.M;
S = length(M);
Z = zeros(S,S);
dydt = [F;zeros(S,1)]-[K,Z;Z,-M]*Q;
end

end