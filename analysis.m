load('data.mat')
t = processdata.t;
x = processdata.x;
xdot = processdata.xdot;
z = processdata.z;
zdot = processdata.zdot;
theta = processdata.theta;
f = controllerdata.actuators.thrust;
w = controllerdata.actuators.pitchrate;
%plot(t,x,t,xdot,t,z,t,zdot,t,theta)
plot(t,f)
%legend('x','xdot','z','zdot','theta')
xlabel('time')
%ylabel('m, m/s, rad')
ylabel('N')
title('3 Second, input minimizing')
grid on