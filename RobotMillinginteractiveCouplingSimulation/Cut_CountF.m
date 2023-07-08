function n_cout_f = Cut_CountF(dw,C_theta)
% This function can generate [count reference] for trajectory storage in tool simulation.
% input:[dw] phase variation of tool in each time step [scalar,rad]
%       [C_theta] initial phase of a tool slice [scalar,negative,rad]
% output: [n_cout_f] count reference of trajectory storage of a tool slice [scalar]

% data calibration
if ~isscalar(dw) || ~isscalar(C_theta)
    error('There may be something wrong!')
end
% Take absolute value of [C_theta]
C_theta = abs(C_theta);

% case 1: if [C_theta] = 0, [count reference] = 0
if C_theta == 0
    n_cout_f = 0;
    return
end

% case 2: if [C_theta] > 0, [count reference] depends on relationship
% between [C_theta] and [dw]. Generally, a [dw] will cover multiple 
% [slice_dphase], so that the [count reference] changes according to 
% [C_theta], which is sum of a series of [slice_dphase]
i = 0;
while 1
    if i*dw > C_theta
        break
    end
    i = i+1;
end
n_cout_f = i;
    
end