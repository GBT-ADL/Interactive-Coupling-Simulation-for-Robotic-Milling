function Tip_p = Cut_TipP(Centre,C_R,Ti_angle)
% This function can calculate position of tip on cutter
% This function is called in class [Cutting_SliceClass]
% input [Centre] Centre of cutter, [col,mm]
%       [C_R] radius of cutter, [scalar,mm]
%       [Ti_angle] phase of teeth i [scalar,rad]

% Input Check
if ~iscolumn(Centre)
    error('Error: Wrong input type')
end
if ~isscalar(C_R)||~isscalar(Ti_angle)
    error('Error: Wrong input type')
end

% Function body
Tip_p = Centre + C_R.*[sin(Ti_angle);-cos(Ti_angle)];
end