% function AnalysisOfOptimalTrajectory
close all
clear all
load traj
topt = t;
xopt = x;
uopt = u;

load data
tsim = processdata.t;
xsim = [processdata.x; processdata.xdot;
        processdata.y; processdata.ydot;
        processdata.z; processdata.zdot; 
        processdata.phi; processdata.phidot;
        processdata.theta; processdata.thetadot
        processdata.psi; processdata.psidot];
usim = [controllerdata.actuators.f1; 
        controllerdata.actuators.f2;
        controllerdata.actuators.f3;
        controllerdata.actuators.f4
        controllerdata.actuators.r];

clf;


figure(1)
subplot(2,1,1)
for i = 1:6
    plot(tsim,xsim(i,:))
    hold on
end
title('Simulation Pos & Vel')
hl = legend('x','$\dot{x}$','y','$\dot{y}$','z','$\dot{z}$','location','northwest');
set(hl, 'Interpreter', 'latex');
grid on
xlim([topt(1) topt(end)]);
subplot(2,1,2)
for i = 1:6
    plot(topt,xopt(i,:))
    hold on
end
title('OptimTraj Pos & Vel')
xlim([topt(1) topt(end)]);
hl = legend('x','$\dot{x}$','y','$\dot{y}$','z','$\dot{z}$','location','northwest');
set(hl, 'Interpreter', 'latex');
grid on

figure(2)
subplot(2,1,1)
for i = 7:12
    plot(tsim,xsim(i,:))
    hold on
end
title('Simulation Angles')
hl = legend('$\phi$','$\dot{\phi}$','$\theta$','$\dot{\theta}$','$\psi$','$\dot{\psi}$','location','southwest');
set(hl, 'Interpreter', 'latex');
xlim([topt(1) topt(end)]);
grid on
subplot(2,1,2)
for i = 7:12
    plot(topt,xopt(i,:))
    hold on
end
title('OptimTraj Angles')

xlim([topt(1) topt(end)]);
hl = legend('$\phi$','$\dot{\phi}$','$\theta$','$\dot{\theta}$','$\psi$','$\dot{\psi}$','location','southwest');
set(hl, 'Interpreter', 'latex');
grid on


figure(3)
subplot(2,1,1)
for i = 1:5
    plot(tsim,usim(i,:))
    hold on
end
title('Simulation Input')
xlim([topt(1) topt(end)]);
grid on
hl = legend('f1','f2','f3','f4','r','location','northwest');
set(hl, 'Interpreter', 'latex');
% xlabel('$\dot{T}$'),ylabel('$\dot{y}');

grid on
subplot(2,1,2)
for i = 1:5
    plot(topt,uopt(i,:))
    hold on
end
title('OptimTraj Input')

xlim([topt(1) topt(end)]);
grid on
hl = legend('f1','f2','f3','f4','r','location','northwest');
set(hl, 'Interpreter', 'latex');

% 
% 
% 
% % Get MATLAB's standard list of colors
% colorList = get(gca,'ColorOrder');
% 
% % Comment from here (State)
% figure(1)
% plot(topt,xopt,'-','linewidth',1);
% hold on;
% for i=1:size(xopt,1)-3
%     plot(tsim,xsim(i,:),'--','color',colorList(i,:));
% end
% title('Quadcoper State')
% legend('x (m)','xdot (m/s)','y (m)','ydot (m/s)','z (m)','zdot (m/s)','location','northwest');
% xlabel('time (s)')
% grid on
% % To here
% xlim([topt(1) topt(end)]);
% 
% figure(2)
% for i=1:3
%     plot(tsim,xsim(i+6,:),'--','color',colorList(i,:));
% end
% title('Quadcoper State')
% legend('theta (rad)','phi (rad)','psi (rad)','location','northwest');
% xlabel('time (s)')
% grid on
% 
%  
% % Comment from here (Input)
% figure(3)
% plot(topt,uopt,'-','linewidth',1);
% hold on;
% for i=1:size(uopt,1)
%     plot(tsim,usim(i,:),'--','color',colorList(i,:));
% end
% legend('pitchrate (rad/s)','thrust (N)','location','northwest');
% title('Quadcopter Inputs')
% xlabel('time (s)')
% grid on
% % To here
% 

% 
% end