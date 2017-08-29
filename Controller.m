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

% Do not modify this function.
func.init = @initControlSystem;
func.run = @runControlSystem;
end


function [actuators,data] = initControlSystem(sensors,references,parameters,data)
clc;
load('runOptions')
syms x xdot y ydot z zdot phi phidot theta thetadot psi psidot f1 f2 f3 f4 real
% Symbolic description of A matrix
A = jacobian(equationsOfMotion,state_sym);
% Symbolic description of B matrix
B = jacobian(equationsOfMotion,inputs_sym);

% Create functions
data.funcA = matlabFunction(A,'Vars',[xdot ydot zdot phi theta state_sym(11) phidot thetadot psidot f1 f2 f3 f4]);
data.funcB = matlabFunction(B,'Vars',[phi theta state_sym(11)]);
data.funcA;
data.funcB;
% Initialize LQR matrices
data.Q = 200*eye(length(A));
sizeB = size(B);
data.R = eye(sizeB(2));


% Trajectory
load('traj.mat')
data.trajT = t;
data.trajX = x(1,:);
data.trajY = x(2,:);
data.trajZ = x(3,:);

data.trajXdot = x(4,:);
data.trajYdot = x(5,:);
data.trajZdot = x(6,:);

data.trajPhi = x(7,:);
data.trajTheta = x(8,:);
data.trajPsi = x(9,:);

data.trajPhidot = x(10,:);
data.trajThetadot = x(11,:);
data.trajPsidot = x(12,:);

data.trajF1 = u(1,:);
data.trajF2 = u(2,:);
data.trajF3 = u(3,:);
data.trajF4 = u(4,:);

data.minThrust = minThrust;
data.maxThrust = maxThrust;

% Analysis variables
data.vel_error = .5;  % Required velocity from rest (for both x and z)
data.pos_error = .5;  % Required distance from final position (for both x and z)
data.endBool = 0;      % Used to only display the end time once


data.index = 1;

% Run the controller
[actuators,data] = runControlSystem(sensors,references,parameters,data);
end


function [actuators,data] = runControlSystem(sensors,references,parameters,data)


% Reference trajectory
ind = data.index;
if data.index <= length(data.trajT)
  Time_diff = abs(data.trajT(data.index)-sensors.t);
    
    data.index = data.index + 1;
    
    trajX = data.trajX(ind);
    trajY = data.trajY(ind);
    trajZ = data.trajZ(ind);
    trajXdot = data.trajXdot(ind);
    trajYdot = data.trajYdot(ind);
    trajZdot = data.trajZdot(ind);
    trajPhi = data.trajPhi(ind);
    trajTheta = data.trajTheta(ind);
    trajPsi = data.trajPsi(ind);
    trajPhidot = data.trajPhidot(ind);
    trajThetadot = data.trajThetadot(ind);
    trajPsidot = data.trajPsidot(ind);
    trajF1 = data.trajF1(ind);
    trajF2 = data.trajF2(ind);
    trajF3 = data.trajF3(ind);
    trajF4 = data.trajF4(ind);
    
else
    
    trajX = data.trajX(end);
    trajY = data.trajY(end);
    trajZ = data.trajZ(end);
    trajXdot = data.trajXdot(end);
    
    trajYdot = data.trajYdot(end);
    
    trajZdot = data.trajZdot(end);
    trajPhi = data.trajPhi(end);
    trajTheta = data.trajTheta(end);
    trajPsi = data.trajPsi(end);
    trajPhidot = data.trajPhidot(end);
    trajThetadot = data.trajThetadot(end);
    trajPsidot = data.trajPsidot(end);
    
    trajF1 = parameters.m*parameters.g/4;
    trajF2 = parameters.m*parameters.g/4;
    trajF3 = parameters.m*parameters.g/4;
    trajF4 = parameters.m*parameters.g/4;

    
end

A = data.funcA(trajXdot, trajYdot, trajZdot, trajPhi,trajTheta, trajPsi,...
               trajPhidot, trajThetadot,trajPsidot, trajF1, trajF2, trajF3, trajF4);
B = data.funcB(trajPhi,trajTheta,trajPsi);

data.K = lqr(A,B,data.Q,data.R);

% Calculate and apply input
state = [sensors.x - trajX; 
         sensors.y - trajY;
         sensors.z - trajZ; 
         
         sensors.xdot - trajXdot;
         sensors.ydot - trajYdot;
         sensors.zdot - trajZdot;
         
         sensors.phi - trajPhi;
         sensors.theta - trajTheta;
         sensors.psi - trajPsi
         
         sensors.phidot - trajPhidot;
         sensors.thetadot - trajThetadot;      
         sensors.psidot - trajPsidot];
    
 
input = -data.K*state + [trajF1; trajF2; trajF3; trajF4];
% input = [trajF1; trajF2; trajF3; trajF4]
% 

% ~~ Correctly plot inputs (Regulated in DesignProblem, but not recorded)~~
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

for i = 1:4
if input(i) < data.minThrust
    input(i) = data.minThrust;
elseif input(i) > data.maxThrust
    input(i) = data.maxThrust;
else
    input(i) = input(i);
end
end


actuators.f1 = input(1);
actuators.f2 = input(2);
actuators.f3 = input(3);
actuators.f4 = input(4);


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