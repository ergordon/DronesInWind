load('data.mat')
t = processdata.t;
x = processdata.x;
xdot = processdata.xdot;
y = processdata.y;
ydot = processdata.ydot;
z = processdata.z;
zdot = processdata.zdot;
theta = processdata.theta;
phi = processdata
f = controllerdata.actuators.thrust;
w = controllerdata.actuators.pitchrate;
%plot(t,x,t,xdot,t,z,t,zdot,t,theta)
plot(t,f)
%legend('x','xdot','z','zdot','theta')
xlabel('Time, t')
%ylabel('m, m/s, rad')
ylabel('Thrust, N')
title('5 Second, time optimal')
grid on