load('runOptions.mat')
syms x xdot y ydot z zdot phi phidot theta thetadot psi psidot f1 f2 f3 f4 r real
state_sym = [x; xdot; y; ydot; z; zdot; phi; phidot; theta; thetadot; psi; psidot];
inputs_sym = [f1 f2 f3 f4 r];
M = [Ix 0 -Ix*sin(theta); ...
    0 Iy*cos(phi)^2+Iz*sin(phi)^2 (Iy-Iz)*cos(phi)*sin(phi)*cos(theta); ...
    -Ix*sin(theta) (Iy-Iz)*cos(phi)*sin(phi)*cos(theta)...
    Ix*sin(theta)^2+Iy*sin(phi)^2*cos(theta)^2+Iz*cos(phi)^2*cos(theta)^2];
c11 = 0;
c12 = (Iy-Iz)*(thetadot*cos(phi)*sin(phi)+psi*sin(phi)^2*cos(theta)) ...
    + (Iz-Iy)*psi*cos(phi)^2*cos(theta) - Ix*psi*cos(theta);
c13 = (Iz-Iy)*psidot*cos(phi)*sin(phi)*cos(theta)^2;
c21 = (Iz-Iy)*(thetadot*cos(phi)*sin(phi)+psi*sin(phi)^2*cos(theta)) ...
    + (Iy-Iz)*psi*cos(phi)^2*cos(theta) + Ix*psi*cos(theta);
c22 = (Iz-Iy)*phidot*cos(phi)*sin(phi);
c23 = -Ix*psidot*sin(theta)*cos(theta) + Iy*psidot*sin(phi)^2*cos(theta)*sin(theta) ...
    + Iz*psidot*cos(phi)^2*sin(theta)*cos(theta);
c31 = (Iy-Iz)*psi*cos(theta)^2*sin(phi)*cos(phi) - Ix*thetadot*cos(theta);
c32 = (Iz-Iy)*(thetadot*cos(phi)*sin(phi)*sin(theta) + phidot*sin(phi)^2*cos(theta)) ...
    + (Iy-Iz)*phidot*cos(phi)^2*cos(theta) + Ix*psidot*sin(theta)*cos(theta) ...
    - Iy*psidot*sin(phi)^2*cos(theta)*sin(theta) - Iz*psidot*cos(phi)^2*sin(theta)*cos(theta);
c33 = (Iy-Iz)*psidot*cos(theta)^2*sin(phi)*cos(phi) - Iy*thetadot*sin(phi)^2*cos(theta)*sin(theta) ...
    - Iz*thetadot*cos(phi)^2*cos(theta)*sin(theta) + Ix*thetadot*cos(theta)*sin(theta);
C = [c11 c12 c13; c21 c22 c23; c31 c32 c33];


T = [(f3-f1)*dim_1; (f4-f2)*dim_2; r];

nddot = inv(M)*(T-C*[phidot; thetadot; psidot]);


pos = [x;y;z];
% [x_n,y_n] = meshgrid(-5:0.1:5,-5:0.1:5);
% u_n = cos(x_n).*y_n;
% v_n = sin(x_n).*y_n;
% figure
% quiver(x_n,y_n,u_n,v_n)
% wind_u = cos(x)*y;
% wind_v = sin(x)*y;

vel = [xdot;ydot;zdot];
A = dim_1*dim_2;
c_d = 0.05;
rho = 1.225;
Fd = (-.5*c_d*rho*A*norm(vel)^2 *vel)/norm(vel);

equationsOfMotion = [xdot; 
                    ((f1+f2+f3+f4)/mass)*(cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi))%+Fd(1)/mass;
                    ydot;
                    ((f1+f2+f3+f4)/mass)*(sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi))%+Fd(2)/mass;
                    zdot; 
                    ((f1+f2+f3+f4)/mass)*cos(theta)*cos(phi)-gravity%+Fd(3)/mass;
                    phidot;
                    nddot(1)
                    thetadot;
                    nddot(2);
                    psidot;
                    nddot(3)];

save('runOptions.mat','equationsOfMotion','state_sym','inputs_sym','-append');