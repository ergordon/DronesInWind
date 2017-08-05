clc; clear;
addpath OptimTraj/
load('runOptions')

maxPitchRate = 5;
maxThrust = 15;
minThrust = 1;
xInitial = 0;
zInitial = 0;
m = 1;
g = 9.81;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Set up function handles                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%


% Put symbolic EOMs in convenient form
syms x xdot z zdot theta thrust w real
f = simplify([xdot; (thrust/m)*sin(theta); zdot; ...
    (thrust/m)*cos(theta) - g; w]);
% Convert EOMs from symbolic to numeric
numf = matlabFunction(f,'vars',[x xdot z zdot theta w thrust]);

problem.func.dynamics = @(t,x,u)( numf(x(1,:), x(2,:), x(3,:), x(4,:), ... 
    x(5,:), u(1,:), u(2,:)) );
%problem.func.pathObj = @(t,x,u)( sum((x(1,:)-xFinal).^2,1) + sum((x(3,:)-zFinal).^2,1) );
%problem.func.pathObj = @(t,x,u)( sum(u.^2,1) );
problem.func.bndObj = @(t0,x0,tF,xF)(tF-t0);


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Set up problem bounds                               %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.bounds.initialTime.low = 0;
problem.bounds.initialTime.upp = 0;
problem.bounds.finalTime.low = 0;
problem.bounds.finalTime.upp = runTime;

problem.bounds.initialState.low = [xInitial; 0; zInitial; 0; 0];
problem.bounds.initialState.upp = [xInitial; 0; zInitial; 0; 0];
problem.bounds.finalState.low = [xFinal; 0; zFinal; 0; 0];
problem.bounds.finalState.upp = [xFinal; 0; zFinal; 0; 0];

% problem.bounds.state.low = [-2*dist;-2*pi;-inf;-inf];
% problem.bounds.state.upp = [2*dist;2*pi;inf;inf];

problem.bounds.control.low = [-maxPitchRate; minThrust];
problem.bounds.control.upp = [maxPitchRate; maxThrust];

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                    Initial guess at trajectory                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.guess.time = [0,runTime];
problem.guess.state = [problem.bounds.initialState.low, problem.bounds.finalState.low];
problem.guess.control = [[0; g],[0; g]];


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Solver options                                  %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.options.nlpOpt = optimset(...
    'Display','iter',...
    'MaxFunEvals',1e5);

% problem.options.method = 'trapezoid';
% problem.options.method = 'hermiteSimpson';
% problem.options.method = 'rungeKutta';
% problem.options.method = 'chebyshev';

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                            Solve!                                       %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

soln = optimTraj(problem);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Display Solution                                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%%%% Unpack the simulation
t = linspace(soln.grid.time(1), soln.grid.time(end), soln.grid.time(end)*timeDensity);
x = soln.interp.state(t);
u = soln.interp.control(t);

clf;
subplot(2,1,1);
plot(t,x);
legend('x','xdot','z','zdot','theta');
subplot(2,1,2);
plot(t,u);
legend('pitch rate','thrust');

save('traj.mat','t','x','u');