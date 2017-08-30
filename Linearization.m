clear all clc
syms x y z xdot ydot zdot  THRUST m g real
syms theta w phi p psi r real
m = 1;
EOMs = [xdot;
        (THRUST/m)*(cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi));
        ydot;
        (THRUST/m)*(sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi))
        zdot;
        (THRUST/m)*cos(theta)*cos(phi)-9.81;
        w;
        p;
        r];
    

State = [x; xdot; y; ydot; z; zdot; theta; phi;psi];
Input = [w;p;r;THRUST];

A = jacobian(EOMs,State)
B = jacobian(EOMs,Input)
funcA = matlabFunction(A);
funcB = matlabFunction(B);

A = funcA (9.81,0,0,0);
B = funcB(0,0,0);

rank(ctrb(A,B));

dimensions = [651 651 188];
A = (dimensions(1)/1000)^2; 
c_d = 0.05;
rho = 1.225; %kg/m^3
% Fd = (-.5*c_d*rho*A*norm(vel)^2 *vel)/norm(vel);

[x,y] = meshgrid(-5:0.1:5,-5:0.1:5);
u = cos(x).*y;
v = sin(x).*y;
figure
quiver(x,y,u,v)