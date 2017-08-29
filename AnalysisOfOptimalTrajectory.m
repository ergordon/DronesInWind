% function AnalysisOfOptimalTrajectory
close all
clear all
load traj
topt = t;
xopt = x;
uopt = u;

load data
tsim = processdata.t;
xsim = [processdata.x; processdata.y; processdata.z;
        processdata.xdot; processdata.ydot; processdata.zdot; 
        processdata.phi; processdata.theta;processdata.psi;
        processdata.phidot; processdata.thetadot; processdata.psidot];
    
usim = [controllerdata.actuators.f1; 
        controllerdata.actuators.f2;
        controllerdata.actuators.f3;
        controllerdata.actuators.f4];

clf;


figure(1)
subplot(2,1,1)
for i = 1:6
    plot(tsim,xsim(i,:))
    hold on
end
title('Simulation Pos & Vel')
hl = legend('x','y','z','$\dot{x}$','$\dot{y}$','$\dot{z}$','location','northwest');
set(hl, 'Interpreter', 'latex');
grid on
xlim([topt(1) topt(end)]);
xlabel('Time (s)')
subplot(2,1,2)
for i = 1:6
    plot(topt,xopt(i,:))
    hold on
end
title('OptimTraj Pos & Vel')
xlim([topt(1) topt(end)]);
xlabel('Time (s)')
hl = legend('x','y','z','$\dot{x}$','$\dot{y}$','$\dot{z}$','location','northwest');
set(hl, 'Interpreter', 'latex');
grid on

figure(2)
subplot(2,1,1)
for i = 7:12
    plot(tsim,xsim(i,:))
    hold on
end
title('Simulation Angles')
hl = legend('$\phi$','$\psi$','$\theta$','$\dot{\phi}$','$\dot{\theta}$','$\dot{\psi}$','location','southwest');
set(hl, 'Interpreter', 'latex');
xlim([topt(1) topt(end)]);
xlabel('Time (s)')
ylim([-2 2])
grid on
subplot(2,1,2)
for i = 7:12
    plot(topt,xopt(i,:))
    hold on
end
title('OptimTraj Angles')
xlabel('Time (s)')
xlim([topt(1) topt(end)]);
hl = legend('$\phi$','$\psi$','$\theta$','$\dot{\phi}$','$\dot{\theta}$','$\dot{\psi}$','location','southwest');
set(hl, 'Interpreter', 'latex');
grid on


figure(3)
subplot(2,1,1)
for i = 1:4
    plot(tsim,usim(i,:))
    hold on
end
title('Simulation Input')
xlabel('Time (s)')
xlim([topt(1) topt(end)]);
% ylim([-10 10])
grid on
hl = legend('f1','f2','f3','f4','location','northwest');


grid on
subplot(2,1,2)
for i = 1:4
    plot(topt,uopt(i,:))
    hold on
end
title('OptimTraj Input')
xlabel('Time (s)')
xlim([topt(1) topt(end)]);
grid on
hl = legend('f1','f2','f3','f4','location','northwest');

