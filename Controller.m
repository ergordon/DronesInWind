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

% Current angle of the quad and current thrust (this will be passed in from the run function)
syms ANG THRUST real

% Symbolic description of A matrix
A = [0 1 0 0 0; 0 0 0 0 THRUST*cos(ANG)/parameters.m; 0 0 0 1 0; ...
    0 0 0 0 -THRUST*sin(ANG)/parameters.m; 0 0 0 0 0];

% Symbolic description of B matrix
B = [0 0; 0 sin(ANG)/parameters.m; 0 0; 0 cos(ANG)/parameters.m; 1 0];

% Create functions
data.funcA = matlabFunction(A);
data.funcB = matlabFunction(B);

% Initialize LQR matrices
data.Q = 700*eye(5);
data.R = eye(2);

% Trajectory
load('traj.mat')
data.trajT = t;
data.trajX = x(1,:);
data.trajXdot = x(2,:);
data.trajZ = x(3,:);
data.trajZdot = x(4,:);
data.trajTheta = x(5,:);
data.trajW = u(2,:);
data.trajF = u(1,:);
data.trajIND = 1;

% Analysis variables
data.stationaryVelocityMargin = .01; % Required velocity from rest (for both x and z)
data.finalPositionMargin = .01; % Required distance from final position (for both x and z)
data.endBool = 0; % Used to only display the end time once

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
    trajW = data.trajW(ind);
    trajF = data.trajF(ind);
else
    data.trajIND = data.trajIND+1;
    trajX = data.trajX(end);
    trajXdot = data.trajXdot(end);
    trajZ = data.trajZ(end);
    trajZdot = data.trajZdot(end);
    trajTheta = data.trajTheta(end);
    trajW = 0;
    trajF = parameters.m * parameters.g;
end

% Linearize
A = data.funcA(trajTheta,trajF);
B = data.funcB(trajTheta);
K = lqr(A,B,data.Q,data.R);

% Calculate and apply input
state = [sensors.x; sensors.xdot; sensors.z; sensors.zdot; sensors.theta] - ...
    [trajX; trajXdot; trajZ; trajZdot; trajTheta];
input = -K*state + [trajW; trajF];
actuators.pitchrate = input(1);
actuators.thrust = input(2);

if abs(sensors.xdot) < data.stationaryVelocityMargin && ...
        abs(sensors.zdot) < data.stationaryVelocityMargin && ...
        abs(sensors.x - data.trajX(end)) < data.finalPositionMargin && ...
        abs(sensors.z - data.trajZ(end)) < data.finalPositionMargin && ...
        data.endBool == 0
    data.endBool = 1;
    data.timeEnd = parameters.tStep*ind;
    fprintf('End State Achieved at: %f seconds\n', data.timeEnd)
end
end