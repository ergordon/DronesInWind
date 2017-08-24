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
% Current angle of the quad and current thrust (this will be passed in from the run function)
syms Psi theta phi f real

% Symbolic description of A matrix
A = [
0, 1, 0, 0, 0, 0,               0,                                                          0,                                0;
0, 0, 0, 0, 0, 0, f*cos(phi)*cos(Psi)*cos(theta),  f*(cos(phi)*sin(Psi)-cos(Psi)*sin(phi)*sin(theta)), f*(cos(Psi)*sin(phi)-cos(phi)*sin(Psi)*sin(theta));
0, 0, 0, 1, 0, 0,               0,                                                          0,                                0;
0, 0, 0, 0, 0, 0, f*cos(phi)*cos(theta)*sin(Psi), -f*(cos(phi)*cos(Psi) + sin(phi)*sin(Psi)*sin(theta)), f*(sin(phi)*sin(Psi) + cos(phi)*cos(Psi)*sin(theta));
0, 0, 0, 0, 0, 1,               0,                                                          0,                                0;
0, 0, 0, 0, 0, 0,      -f*cos(phi)*sin(theta),                                -f*cos(theta)*sin(phi),                         0;
0, 0, 0, 0, 0, 0,               0,                                                          0,                                0;
0, 0, 0, 0, 0, 0,               0,                                                          0,                                0;
0, 0, 0, 0, 0, 0,               0,                                                          0,                                0];
% Symbolic description of B matrix
B = [
0, 0, 0,                   0;
0, 0, 0, sin(phi)*sin(Psi) + cos(phi)*cos(Psi)*sin(theta);
0, 0, 0,                   0;
0, 0, 0, cos(phi)*sin(Psi)*sin(theta) - cos(Psi)*sin(phi);
0, 0, 0,                   0;
0, 0, 0,           cos(phi)*cos(theta);
1, 0, 0,                   0;
0, 1, 0,                   0;
0, 0, 1,                   0];
 
% Create functions
data.funcA = matlabFunction(A);     %(f,phi,Psi,theta)
data.funcB = matlabFunction(B);     %(phi,Psi,theta)

% Initialize LQR matrices
data.Q = 200*eye(9);
data.R = eye(4);

% Trajectory
load('traj.mat')
data.trajT = t;
data.trajX = x(1,:);
data.trajXdot = x(2,:);
data.trajY = x(3,:);
data.trajYdot = x(4,:);
data.trajZ = x(5,:);
data.trajZdot = x(6,:);
data.trajTheta = x(7,:);
data.trajPhi = x(8,:);
data.trajPsi = x(9,:);

data.trajW = u(1,:);
data.trajP = u(2,:);
data.trajR = u(3,:);
data.trajF = u(4,:);

load('runOPtions.mat')
data.minThrust = minThrust;
data.maxThrust = maxThrust;
data.maxPitchRate = maxPitchRate;
data.maxRollRate = maxRollRate;
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
%     Time_diff = abs(data.trajT(data.index)-sensors.t)
    
    data.index = data.index + 1;    
    
    trajX = data.trajX(ind);
    trajXdot = data.trajXdot(ind);
    trajY = data.trajY(ind);
    trajYdot = data.trajYdot(ind);
    trajZ = data.trajZ(ind);
    trajZdot = data.trajZdot(ind);
    trajTheta = data.trajTheta(ind);
    trajPhi = data.trajPhi(ind);
    trajPsi = data.trajPsi(ind);
    
    trajW = data.trajW(ind);
    trajP = data.trajP(ind);
    trajR = data.trajR(ind);
    trajF = data.trajF(ind);
else
    
    trajX = data.trajX(end);
    trajXdot = data.trajXdot(end);
    trajY = data.trajY(end);
    trajYdot = data.trajYdot(end);
    trajZ = data.trajZ(end);
    trajZdot = data.trajZdot(end);
    trajTheta = data.trajTheta(end);
    trajPhi = data.trajPhi(end);
    trajPsi = data.trajPsi(end);
%     
    
    trajW = 0;
    trajP = 0;
    trajR = 0;
    trajF = parameters.m*parameters.g;
    
end

A = data.funcA(trajPsi,trajF,trajPhi,trajTheta);
B = data.funcB(trajPsi,trajPhi,trajTheta);
data.K = lqr(A,B,data.Q,data.R);

% Calculate and apply input
state = [sensors.x - trajX; 
         sensors.xdot - trajXdot;
         sensors.y - trajY;
         sensors.ydot - trajYdot;
         sensors.z - trajZ; 
         sensors.zdot - trajZdot; 
         sensors.theta - trajTheta;
         sensors.phi - trajPhi;
         sensors.psi - trajPsi];
    
 
input = -data.K*state + [trajW; trajP; trajR; trajF];
% input = [trajW; trajP; trajR; trajF];



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

actuators.pitchrate = input(1);
actuators.rollrate = input(2);
actuators.yawrate = input(3);
actuators.thrust = input(4);


% Display when the quad has reached the goal state
if abs(sensors.xdot) < data.vel_error && ...
        abs(sensors.zdot) < data.vel_error && ...
        abs(sensors.x - data.trajX(end)) < data.pos_error && ...
        abs(sensors.y - data.trajY(end)) < data.pos_error && ...
        abs(sensors.z - data.trajZ(end)) < data.pos_error && ...
        data.endBool == 0
    data.endBool = 1;
    data.timeEnd = sensors.t;
    position = [sensors.x,sensors.y sensors.z]
    fprintf('End State Achieved at: %f seconds\n', data.timeEnd)
    
end


end