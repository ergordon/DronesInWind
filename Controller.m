function func = Controller
close all
% INTERFACE
%
%   sensors
%       .x          (x position)
%       .xdot       (x velocity)
%       .z          (z position)
%       .zdot       (z velocity)
%       .phi        (pitch angle)
%       .theta      (roll angle)
%       .psi        (yaw angle)
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
%       .f1             (Thrust of first rotor)
%       .f2             (Thrust of second rotor)
%       .f3             (Thrust of third rotor)
%       .f4             (Thrust of fourth rotor)
%       .r              (Yaw Moment)

% Do not modify this function.
func.init = @initControlSystem;
func.run = @runControlSystem;
end


function [actuators,data] = initControlSystem(sensors,references,parameters,data)
clc;
load('runOptions')
syms x xdot y ydot z zdot phi phidot theta thetadot psi psidot f1 f2 f3 f4 r real
% Symbolic description of A matrix
A = jacobian(equationsOfMotion,state_sym);
% Symbolic description of B matrix
B = jacobian(equationsOfMotion,inputs_sym);

% Create functions
data.funcA = matlabFunction(A,'Vars',[xdot ydot zdot phi phidot theta thetadot state_sym(11) psidot f1 f2 f3 f4 r]);
data.funcB = matlabFunction(B,'Vars',[phi theta state_sym(11)]);

% Initialize LQR matrices
data.Q = 200*eye(length(A));
sizeB = size(B);
data.R = eye(sizeB(2));

% Trajectory
load('traj.mat')
data.trajT = t;
data.trajX = x(1,:);
data.trajXdot = x(2,:);
data.trajY = x(3,:);
data.trajYdot = x(4,:);
data.trajZ = x(5,:);
data.trajZdot = x(6,:);
data.trajPhi = x(7,:);
data.trajPhidot = x(8,:);
data.trajTheta = x(9,:);
data.trajThetadot = x(10,:);
data.trajPsi = x(11,:);
data.trajPsidot = x(12,:);

data.trajF1 = u(1,:);
data.trajF2 = u(2,:);
data.trajF3 = u(3,:);
data.trajF4 = u(4,:);
data.trajR = u(5,:);

data.minThrust = minThrust;
data.maxThrust = maxThrust;
data.maxYawRate = maxYawRate;

% Analysis variables
data.vel_error = .5;  % Required velocity from rest (for both x and z)
data.pos_error = .05;  % Required distance from final position (for both x and z)
data.endBool = 0;      % Used to only display the end time once


data.index = 1;

% Run the controller
[actuators,data] = runControlSystem(sensors,references,parameters,data);
end


function [actuators,data] = runControlSystem(sensors,references,parameters,data)


% Reference trajectory
ind = data.index;
if data.index < length(data.trajT)
%   Time_diff = abs(data.trajT(data.index)-sensors.t)
    
    data.index = data.index + 1;    
    
    trajX = data.trajX(ind);
    trajXdot = data.trajXdot(ind);
    trajY = data.trajY(ind);
    trajYdot = data.trajYdot(ind);
    trajZ = data.trajZ(ind);
    trajZdot = data.trajZdot(ind);
    trajPhi = data.trajPhi(ind);
    trajPhidot = data.trajPhidot(ind);
    trajTheta = data.trajTheta(ind);
    trajThetadot = data.trajThetadot(ind);
    trajPsi = data.trajPsi(ind);
    trajPsidot = data.trajPsidot(ind);
    
    trajF1 = data.trajF1(ind);
    trajF2 = data.trajF2(ind);
    trajF3 = data.trajF3(ind);
    trajF4 = data.trajF4(ind);
    trajR = data.trajR(ind);
else
    
    trajX = data.trajX(end);
    trajXdot = data.trajXdot(end);
    trajY = data.trajY(end);
    trajYdot = data.trajYdot(end);
    trajZ = data.trajZ(end);
    trajZdot = data.trajZdot(end);
    trajPhi = data.trajPhi(end);
    trajPhidot = data.trajPhidot(end);
    trajTheta = data.trajTheta(end);
    trajThetadot = data.trajThetadot(end);
    trajPsi = data.trajPsi(end);
    trajPsidot = data.trajPsidot(end);
    
    trajF1 = parameters.m*parameters.g/4;
    trajF2 = parameters.m*parameters.g/4;
    trajF3 = parameters.m*parameters.g/4;
    trajF4 = parameters.m*parameters.g/4;
    trajR = 0;
    
end

A = data.funcA(trajXdot, trajYdot, trajZdot, trajPhi, trajPhidot, trajTheta, trajThetadot, trajPsi, trajPsidot, trajF1, trajF2, trajF3, trajF4, trajR);
B = data.funcB(trajPhi,trajTheta,trajPsi);

data.K = lqr(A,B,data.Q,data.R);

% Calculate and apply input
state = [sensors.x - trajX; 
         sensors.xdot - trajXdot;
         sensors.y - trajY;
         sensors.ydot - trajYdot;
         sensors.z - trajZ; 
         sensors.zdot - trajZdot; 
         sensors.phi - trajPhi;
         sensors.phidot - trajPhidot;
         sensors.theta - trajTheta;
         sensors.thetadot - trajThetadot;
         sensors.psi - trajPsi
         sensors.psidot - trajPsidot];
    
 
input = -data.K*state + [trajF1; trajF2; trajF3; trajF4; trajR];



%~~ Correctly plot inputs (Regulated in DesignProblem, but not recorded)~~
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% % Regulate Pitch Rate
% if input(1) < -data.maxPitchRate
%     input(1) = -data.maxPitchRate;
% elseif input(1) > data.maxPitchRate
%     input(1) = data.maxPitchRate;
% else
%     input(1) = input(1);
% end
% 
% % Regulate Roll Rate
% if input(2) < -data.maxRollRate
%     input(2) = -data.maxRollRate;
% elseif input(1) > data.maxRollRate
%     input(2) = data.maxRollRate;
% else
%     input(2) = input(2);
% end
% 
% % Regulate Yaw Rate
% if input(3) < -data.maxYawRate
%     input(3) = -data.maxYawRate;
% elseif input(3) > data.maxYawRate
%     input(3) = data.maxYawRate;
% else
%     input(3) = input(3);
% end
% 
% 
% % Regulate Thrust
% if input(4) < data.minThrust
%     input(4) = data.minThrust;
% elseif input(4) > data.maxThrust
%     input(4) = data.maxThrust;
% else
%     input(4) = input(4);
% end
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

actuators.f1 = input(1);
actuators.f2 = input(2);
actuators.f3 = input(3);
actuators.f4 = input(4);
actuators.r = input(5);


% Display when the quad has reached the goal state
if abs(sensors.xdot) < data.vel_error && ...
        abs(sensors.ydot) < data.vel_error && ...
        abs(sensors.zdot) < data.vel_error && ...
        abs(sensors.x - data.trajX(end)) < data.pos_error && ...
        abs(sensors.y - data.trajY(end)) < data.pos_error && ...
        abs(sensors.z - data.trajZ(end)) < data.pos_error && ...
        data.endBool == 0
    data.endBool = 1;
    data.timeEnd = sensors.t;
    position = [sensors.x,sensors.y sensors.z];
    fprintf('End State Achieved at: %f seconds\n', data.timeEnd)
    
end


end