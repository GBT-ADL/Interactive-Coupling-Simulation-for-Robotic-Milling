function [dt,num] = Cut_Simudt(w,n_cuttip,num)
% Design length of each time step [simu_dt] for Cutting process simulation
% Density of sampling points is controled by this function.
% input: [w] spendle speed [scalar,rad/s]
%        [n_cuttip] number of tips on cutting tool [scalar]
%        [num] number of time steps in a period [scalar]
% output: [dt] length of each time step [scalar,s]
%         [num] also number of time steps in a period, must be int and
%         integrad multiple of [n_cuttip] [scalar]

% data calibration
if ~isscalar(w)||~isscalar(num)||~isscalar(n_cuttip)
    error('There may be something wrong!')
end

num = int64(num/n_cuttip)*n_cuttip; % num must be int and integral multiple of [n_cuttip] [scalar,int]
num = double(num); % num is transformed into double [scalar,double]
dtheta = 2*pi/num; % value of centre angle in each time step [scalar,rad]
dt = dtheta/w; % length of each time step [simu_dt] [scalar,s]

end