function [q,qd] = Robot_LTraj(Robot, position,feed_speed,Robot_R)
% This function can generate a uniform linear trajectory for robot

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

% During milling process, orientation of frame TCP is fixed, its
% - x axis of [TCP frame] pointing up
% - z axis of [TCP frame] is feed direction
% trajectory of robot's TCP is a spacial uniform linear motion along 
% z-axis, feed speed is constant. if a point on trajectory is given, 
% corresponding q,qd and qdd can be calculated.

% input: [Robot] robot model
%        [Position]=[x,y,z] in inertia frame[row,m]
%        [feed speed] feed speed [scalar,m/s]
%        [Robot_R] rotation matrix between inertia frame and TCP frame 
% output: [q,qd] current generalized coordinate and its differentials 
%         they are 6d [column vectors].

% Input calibration
if ~isrow(position)||~isscalar(feed_speed)
    error('Error: Wrong input type')
end

% Feed speed in cartesain coordinate system form
% reference frame is [inertia frame]
feed_speed_cartesain = [feed_speed,0,0];

% [q] Calculation
q = Robot.ikine(transl(position)*Robot_R);
% Robot.plot(q,'workspace',[-4 4 -4 4 -2 4])    
% hold on
% Robot.plot(q,'workspace',[-4,4,-4,4,-2,4])    
% q(q<1e-10) = 0; % neglect magnitudes < 1e-5
% Note that filter of q is designed for robot vibration simulation at long
% long ago, due to its influence on robotic milling simulation, this term
% is cancelled.
Robot_J = Robot.jacob0(q);
Jacb_upper = Robot_J (1:3,:);

% [qd] Calculation
qd = (Jacb_upper\feed_speed_cartesain.').';

% Column vector generation
q = q.';
qd = qd.';
end 

