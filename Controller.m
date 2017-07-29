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

%
% STEP #1: Modify, but do NOT rename, this function. It is called once,
% before the simulation loop starts.
%

function [actuators,data] = initControlSystem(sensors,references,parameters,data)


data.weight = parameters.m*parameters.g;
syms theta real
   
A = [0, 1, 0, 0, 0;
     0, 0, 0, 0, parameters.g*cos(theta)/parameters.m;
     0, 0, 0, 1, 0;
     0, 0, 0, 0, -parameters.g*sin(theta)/parameters.m;
     0, 0, 0, 0, 0];

 B = [0, 0;
      0, sin(theta)/parameters.m;
      0, 0;
      0, cos(theta)/parameters.m;
      1, 0];

data.funcA = matlabFunction(A);
data.funcB = matlabFunction(B);

load('optimal.mat');
data.T = t;
data.X = x(1,:);
data.Xdot = x(2,:);
data.Z = x(3,:);
data.Zdot = x(4,:);
data.Theta = x(5,:);


% Initialize
data.index = 1;
data.x_eq = zeros(5,1);
data.Bool = 0;

[actuators,data] = runControlSystem(sensors,references,parameters,data);
end

%
% STEP #2: Modify, but do NOT rename, this function. It is called every
% time through the simulation loop.
%

function [actuators,data] = runControlSystem(sensors,references,parameters,data)

x = [sensors.x;
     sensors.xdot;
     sensors.z;
     sensors.zdot;
     sensors.theta];

 

 
if data.index <= length(data.T)
    data.x_eq = [data.X(data.index);
                 data.Xdot(data.index);
                 data.Z(data.index);
                 data.Zdot(data.index);
                 data.Theta(data.index)];
             
    data.index = data.index + 1;
end


X = x-data.x_eq;


theta = data.x_eq(5,1);
A = data.funcA(theta);
B = data.funcB(theta);
Q = 3000*eye(5);
R = eye(2);
K = lqr(A,B,Q,R);

u = -K*X;

actuators.thrust = u(2) + data.weight; 
actuators.pitchrate = u(1); 


if abs(sensors.xdot) < .01 && ...
        abs(sensors.zdot) < .01 && ...
        abs(sensors.x - data.X(end)) < .01 && ...
        abs(sensors.z - data.Z(end)) < .01 && ...
        data.Bool == 0
    data.Bool = 1;    
    fprintf('End State Achieved at: %f seconds\n', sensors.t)
end



end
