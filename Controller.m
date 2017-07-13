function func = Controller
% INTERFACE
%
%   sensors
%       .x          (x position)
%       .xdot       (x velocity)
%       .z          (z position)
%       .zdot       (z velocity)
%       .theta      (pitch angle)
%
%   references
%       
%
%   parameters
%       .tStep          (time step)
%       .m              (mass)
%       .g              (acceleration of gravity)
%       .maxthrust      (maximum thrust)
%       .maxpitchrate   (maximum pitch rate)
%
%   data
%       .whatever       (yours to define - put whatever you want into "data")
%
%   actuators
%       .pitchrate      (pitch angular rate)
%       .thrust         (net thrust)

% Do not modify this function.
func.init = @initControlSystem;
func.run = @runControlSystem;
end


function [actuators,data] = initControlSystem(sensors,references,parameters,data)

% Current angle of the quad (this will be passed in from the run function)
syms ANG real

% Symbolic description of A matrix
A = [0 1 0 0 0; 0 0 0 0 parameters.g*cos(ANG); 0 0 0 1 0; ...
    0 0 0 0 -parameters.g*sin(ANG); 0 0 0 0 0];

% Symbolic description of B matrix
B = [0 0; 0 sin(ANG)/parameters.m; 0 0; 0 cos(ANG)/parameters.m; 1 0];

% Create functions
data.funcA = matlabFunction(A);
data.funcB = matlabFunction(B);

% Initialize LQR matrices
data.Q = 2000*eye(5);
data.R = eye(2);

% Trajectory
load('traj.mat')
data.trajT = t;
data.trajX = x(1,:);
data.trajXdot = x(2,:);
data.trajZ = x(3,:);
data.trajZdot = x(4,:);
data.trajTheta = x(5,:);
data.trajIND = 1;

% Run the controller
[actuators,data] = runControlSystem(sensors,references,parameters,data);
end


function [actuators,data] = runControlSystem(sensors,references,parameters,data)

% Reference trajectory
ind = data.trajIND;
if (data.trajIND<length(data.trajT))
    data.trajIND = data.trajIND+1;
    trajX = data.trajX(ind);
    trajXdot = data.trajXdot(ind);
    trajZ = data.trajZ(ind);
    trajZdot = data.trajZdot(ind);
    trajTheta = data.trajTheta(ind);
else
    trajX = data.trajX(ind);
    trajXdot = data.trajXdot(ind);
    trajZ = data.trajZ(ind);
    trajZdot = data.trajZdot(ind);
    trajTheta = data.trajTheta(ind);
end

% Linearize
ANG = trajTheta;
A = data.funcA(ANG);
B = data.funcB(ANG);
K = lqr(A,B,data.Q,data.R);

% Calculate and apply input
state = [sensors.x; sensors.xdot; sensors.z; sensors.zdot; sensors.theta] - ...
    [trajX; trajXdot; trajZ; trajZdot; trajTheta];
input = -K*state;
actuators.thrust = input(2) + parameters.g;
actuators.pitchrate = input(1);

if (sensors.xdot==0 && sensors.zdot == 0)
    data.timeEnd = t
end
end