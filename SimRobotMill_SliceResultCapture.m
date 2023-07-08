%% Milling simulatino and SLD drawing
% milling situation under a list of operation parameters couples are
% implemented in this program, the results will be also used in SLD
% drawing.

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% -----Slice cutting simulation----- %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tic
Spindle_speed_op = 2000; % desired spindle speed op [scalar,rpm]
Spindle_speed_ed = 8000; % desired spindle speed ed [scalar,rpm]
Spindle_speed_interval = 100; % variation between adjacent SS [scalar,rpm]
SS = Spindle_speed_op: Spindle_speed_interval: Spindle_speed_ed; % sequency of SS [row]
L_SS = length(SS); % num of Spindle speed [scalar]


Axial_depth_op = 0.5; % desired axial depth op [scalar,mm]
Axial_depth_ed = 2.2; % desired axial depth ed [scalar,mm]
axial_interval = 0.1; % variation between adjacent axial depth [scalar,mm]
AP = Axial_depth_op:axial_interval:Axial_depth_ed; % sequency of AP [row]
L_AP = length(AP); % num of axial depth of cut [scalar]

disp('the simulated SS and AP are:')
disp(SS)
disp(AP)
while 1
    f = input('Findest du in ordnung? ');
    if f == 1;break;else;error('请提高从者练度，或更换合适的强化礼装');end
end

for j = Axial_depth_op:axial_interval:Axial_depth_ed
    parfor i = 1:L_SS
        SP = SS(i); % current spindle speed [scalar,rpm,modify]
        AD = j; % current axial of depth [scalar,mm]
        name = SimRobotMill_KUKAODE_Slice_function(SP,AD);
        disp([name,' is over!'])
    end
end

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% --------Information storage------- %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
save('Simulation_parameters.mat','Spindle_speed_op','Spindle_speed_ed', ...
    'Spindle_speed_interval','Axial_depth_op','Axial_depth_ed',...
    'axial_interval')
toc
% 5*10 = 50 pairs operation parameters 50 periods 160 points 76min = 1h16min
% 4*31 = 124 pairs operation parameter 50 periods 160 points 2h38min = 158min
% 51*21 = 1071 pairs operation parameters 0.5s 144 points/T 20h55min = 1255min 
% 46*17 = 782 pairs operation parameters 0.5s 120 points/T 13h34min = 815min
% 46*17 = 782 pairs operation parameters 0.5s 144 points/T 15h37min = 937min
% 54*21 = 1134 pairs operation parameters 0.5s 144 points/T 24h21min = 1462min
% 54*21 = 1134 paris operation parameters 0.1s 120 points/T 5h57min= 357min
% 81*41 = 3321 pairs operation parameters 0.5s 144 points/T 9h45min = 585min 
% 81*21 = 1701 pairs operation parameters 0.5s 120 points/T 4h36min = 276min
% 81*21 = 1701 pairs operation parameters 0.5s 160 points/T 6h12min = 372min
% 161*21 = 3381 pairs operation parameters 0.5s 144 points/T 10h02min = 602min
% 81*21 = 1701 pairs operation parameters 1s 160 points/T 14h30min = 870min









