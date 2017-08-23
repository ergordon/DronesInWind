clc; clear;
addpath OptimTraj/
load('runOptions')

xInitial = 0;
zInitial = 0;
yInitial = 0;

m = mass;
g = gravity;

dimensions = [651 651 188];
A = (dimensions(1)/1000)^2; 
c_d = 0.05;
rho = 1.225; %kg/m^3

% [x_n,y_n] = meshgrid(-5:0.1:5,-5:0.1:5);
% u_n = cos(x_n).*y_n;
% v_n = sin(x_n).*y_n;
% figure
% quiver(x_n,y_n,u_n,v_n)


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Set up function handles                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%



% Put symbolic EOMs in convenient form
syms x xdot y ydot z zdot phi theta psi phidot thetadot psidot ...
    f1 f2 f3 f4 r real

vel = [xdot;ydot;zdot];
pos = [x;y;z];

% wind_u = cos(x)*y;
% wind_v = sin(x)*y;

Fd = (-.5*c_d*rho*A*norm(vel)^2 *vel)/norm(vel);
% Fd = [0;0;0]

f = simplify([xdot; 
              ((f1+f2+f3+f4)/m)*(cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi))+Fd(1)/m; %+wind_u; 
              ydot;
              ((f1+f2+f3+f4)/m)*(sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi))+Fd(2)/m; %+wind_v;
              zdot; 
              ((f1+f2+f3+f4)/m)*cos(theta)*cos(phi)-g+Fd(3)/m;
              phidot;
              nddot(1)
              thetadot;
              nddot(2)
              psidot;
              nddot(3)]);
              
% Convert EOMs from symbolic to numeric
numf = matlabFunction(f,'vars',[x xdot y ydot z zdot phi phidot theta...
    thetadot psi psidot f1 f2 f3 f4 r]);

problem.func.dynamics = @(t,x,u)( numf(x(1,:), x(2,:), x(3,:), x(4,:), x(5,:),...
                                       x(6,:), x(7,:), x(8,:), x(9,:),...
                                       x(10,:), x(11,:), x(12,:),...
                                       u(1,:), u(2,:), u(3,:), u(4,:), u(5,:) ));
                                   
                                   
                                   
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

problem.bounds.initialState.low = [xInitial; 0; yInitial; 0; zInitial; 0;...
                                    0; 0; 0; 0; 0; 0];
problem.bounds.initialState.upp = [xInitial; 0; yInitial; 0; zInitial; 0;...
                                    0; 0; 0; 0; 0; 0];
problem.bounds.finalState.low = [xFinal; 0; yFinal; 0; zFinal; 0;...
                                    0; 0; 0; 0; 0; 0];
problem.bounds.finalState.upp = [xFinal; 0; yFinal; 0; zFinal; 0;...
                                    0; 0; 0; 0; 0; 0];


problem.bounds.control.low = [minThrust; minThrust; minThrust; minThrust; -maxPitchRate];
problem.bounds.control.upp = [maxThrust; maxThrust; maxThrust; maxThrust; maxPitchRate];

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                    Initial guess at trajectory                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.guess.time = [0,runTime];
problem.guess.state = [problem.bounds.initialState.low, problem.bounds.finalState.low];
problem.guess.control = [[m*g/4; m*g/4; m*g/4; m*g/4; 0], [m*g/4; m*g/4; m*g/4; m*g/4; 0]];


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
plot(t,x(1,:),t,x(3,:),t,x(5,:));
legend('x','y','z');
subplot(2,1,2)
plot(t,x(2,:),t,x(4,:),t,x(6,:));
legend('xdot','ydot','zdot');


figure(2)
subplot(2,1,1)
plot(t,x(7,:),t,x(9,:),t,x(11,:));
legend('\theta','\phi','\psi')
subplot(2,1,2);
plot(t,u);
legend('f1','f2','f3','f4','yaw rate');

save('traj.mat','t','x','u');