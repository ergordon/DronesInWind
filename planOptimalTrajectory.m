clc; clear;
addpath OptimTraj/
load('runOptions')

xInitial = 0;
zInitial = 0;
yInitial = 0;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Set up function handles                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% Put symbolic EOMs in convenient form
syms x xdot y ydot z zdot phi theta psi phidot thetadot psidot ...
    f1 f2 f3 f4 real

% Convert EOMs from symbolic to numeric
numf = matlabFunction(simplify(equationsOfMotion),'vars',[x y z...
                                                          xdot ydot zdot...
                                                          phi theta psi...
                                                          phidot thetadot psidot...
                                                          f1 f2 f3 f4]);
problem.func.dynamics = @(t,x,u)( numf(x(1,:), x(2,:), x(3,:),...
                                       x(4,:), x(5,:), x(6,:),...
                                       x(7,:), x(8,:), x(9,:),...
                                       x(10,:), x(11,:), x(12,:),...
                                       u(1,:), u(2,:), u(3,:), u(4,:) ));
                                   
                                   
                                   
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%          
%                           Optimize for:                                     %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% Input:
%	problem.func.pathObj = @(t,x,u)( sum(u.^2,1) );

% Thrust:
%	problem.func.pathObj = @(t,x,u)( sum(u(2,:).^2,1) );

% Pitch Rate:
%   problem.func.pathObj = @(t,x,u)( sum(u(1,:).^2,1) );

 % Time
%   problem.func.pathObj = @(t,x,u)( sum((x(1,:)-x_f).^2,1) + sum((x(3,:)-z_f).^2,1))
problem.func.bndObj = @(t0,x0,tF,xF)(tF-t0);


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Set up problem bounds                               %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.bounds.initialTime.low = 0;
problem.bounds.initialTime.upp = 0;
problem.bounds.finalTime.low = 0;
problem.bounds.finalTime.upp = runTime;

problem.bounds.initialState.low = [xInitial; yInitial; zInitial; 0; 0; 0;...
                                    0; 0; 0; 0; 0; 0];
problem.bounds.initialState.upp = [xInitial; yInitial; zInitial; 0; 0; 0;...
                                    0; 0; 0; 0; 0; 0];
problem.bounds.finalState.low = [xFinal; yFinal; zFinal; 0; 0; 0;...
                                    0; 0; 0; 0; 0; 0];
problem.bounds.finalState.upp = [xFinal; yFinal; zFinal; 0; 0; 0;...
                                    0; 0; 0; 0; 0; 0];

problem.bounds.control.low = [minThrust; minThrust; minThrust; minThrust];
problem.bounds.control.upp = [maxThrust; maxThrust; maxThrust; maxThrust];

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                    Initial guess at trajectory                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.guess.time = [0,runTime];
problem.guess.state = [problem.bounds.initialState.low, problem.bounds.finalState.low];
problem.guess.control = [[mass*gravity/4; mass*gravity/4; mass*gravity/4; mass*gravity/4], [mass*gravity/4; mass*gravity/4; mass*gravity/4; mass*gravity/4]];


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Solver options                                  %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.options.nlpOpt = optimset(...
    'Display','iter',...
    'MaxFunEvals',1e6);
problem.options.MaxFunctionEvaluations = 100000
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
figure(1)
subplot(2,1,1);
plot(t,x(1,:),t,x(2,:),t,x(3,:));
legend('x','y','z');
grid on
subplot(2,1,2)
plot(t,x(4,:),t,x(5,:),t,x(6,:));
legend('xdot','ydot','zdot');
grid on


figure(2)
subplot(2,1,1)
plot(t,x(7,:),t,x(8,:),t,x(9,:));
legend('\theta','\phi','\psi')
grid on
subplot(2,1,2);
plot(t,x(10,:),t,x(11,:),t,x(12,:));
legend('\thetadot','\phidot','\psidot')
grid on

figure(3)
plot(t,u);
legend('f1','f2','f3','f4');
grid on

save('traj.mat','t','x','u');