load('runOptions.mat')
syms x xdot y ydot z zdot phi phidot theta thetadot psi psidot f1 f2 f3 f4 w1 w2 w3 w4 r real
state_sym = [x; xdot; y; ydot; z; zdot; phi; phidot; theta; thetadot; psi; psidot];
inputs_sym = [f1 f2 f3 f4];

I = [Ix 0 0; 0 Iy 0; 0 0 Iz];
Rw = [1 0 -sin(theta); 
           0 cos(phi) cos(theta)*sin(phi); 
           0 -sin(phi) cos(theta)*cos(phi)];
omega_b = Rw * [phidot;thetadot;psidot];



% k = (Kv*Kt*sqrt(2*rho*A)/Kt)^2
% b = .5*R*rho*Cd*A*R^2
% Quadcopter, Dynamics, Simulation, and Control
k = 1;
b = 1;
% f1 = k*w1^2;
% f2 = k*w2^2;
% f3 = k*w3^2;
% f4 = k*w4^2;
Tb = [0;0;f1+f2+f3+f4];
Rz = [cos(psi) -sin(psi) 0;
      sin(psi) cos(psi) 0;
      0 0 1];

Ry = [cos(theta) 0 sin(theta);
      0 1 0;
      -sin(theta) 0 cos(theta)];
  
Rx = [1 0 0;
      0 cos(phi) -sin(phi);
      0 sin(phi) cos(phi)];

% Rotation Matrix  
R= Rz*Ry*Rx;  
  

tau_phi = k*(f3-f1)*dim_1;
tau_theta = k*(f4-f2)*dim_2;
tau_psi = b*(f1-f2+f3-f4);

Tau_b = [tau_phi; tau_theta; tau_psi];


equationsOfMotion = [xdot;ydot;zdot;
                    ([0;0;-gravity]+R*Tb/mass);
                    (inv(Rw)*[phidot;thetadot;psidot]);
                     inv(I)*Tau_b-cross(omega_b,I*omega_b)]
    
pos = [x;y;z];


save('runOptions.mat','equationsOfMotion','state_sym','inputs_sym','-append');