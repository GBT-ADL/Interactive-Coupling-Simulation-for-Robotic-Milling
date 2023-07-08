%% Milling simulatino and SLD drawing
% milling situation under a list of operation parameters couples are
% implemented in this program, the results will be also used in SLD
% drawing.

% 202303: milling simulation in ideal situation is simulated 
% 20230329: feed speed is modified as 1 mm/s, but the result is bad

%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ------Tool cutting simulation----- %
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
        % name = SimRobotMill_KUKAODE_Tool_function(SP,AD); % robotic milling simulation
        name = SimRobotMill_KUKAODE_Tool_function(SP,AD); % ideal milling simulation
        disp([name,' is over!'])
    end
end

%%            
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% --------Information storage------- %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
save('CFSimulationResult\Simulation_parameters.mat','Spindle_speed_op','Spindle_speed_ed', ...
    'Spindle_speed_interval','Axial_depth_op','Axial_depth_ed',...
    'axial_interval')
% the .mat document [Simulation_parameters.mat] should be copied to the
% file in which the simulation results are stored.
toc
