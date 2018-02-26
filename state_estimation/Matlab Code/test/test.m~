%% Initialization
% Units in cm
Lx = 43;
Ly = 33;
Cv = 12;
Cr = 7;
x = [12 3 0];
%% One Iteration
% Generate process noise and measurment noise
v = wgn(3,1,1); % white Gaussian with var = 1
w = wgn(3,1,1);
R = cov(v,v) 
Q = cov(w,w)
% Sensor Measurement Problem
l0 = [(Lx - x(1)) / cos(x(3)), (x(2)/cos(x(3)))];
[H, C] = hmatrix(x, l0, Lx, Ly);
% State Evolution Problem
p_str = (u(1) * u(2)) >= 0;
A = eye(3);
B = bmatrix(p_str, Cv, Cr, x(3));

%% Calculate State
y = H*x.' + C;
u = [1 1];
x_next = A*x.' + B*u.'