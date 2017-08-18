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
funcA
funcB = matlabFunction(B);
funcB
A = funcA (9.81,0,0,0);
B = funcB(0,0,0);

rank(ctrb(A,B))

% A = [0 1 0 0 0 0 0;0 0 0 0 0 0 9.81;0 0 0 1 0 0 0;0 0 0 0 0 0 -9.81;0 0 0 0 0 1 0;0 0 0 0 0 0 0;0 0 0 0 0 0 0]
% B = [0 0;0 0;0 0;0 0;0 0;0 1;1 0];