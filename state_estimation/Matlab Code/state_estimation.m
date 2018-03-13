clear all; close all; clc;
%% Initialization
% Constants in mm
Lx = 490;
Ly = 360;
Cv = 147;
Cr = 90/0.5 * (pi/180);
x_init = [140; 30; 0];
% Generate process and measurment noise covariance matrix
v = [];
for i=1:1000
    v = [v, wgn(3,1,1)]; 
end
R = 0.1* eye(3); %measurement noise
% R = 1/999 * v * v.';
Q = zeros(3); %process noise
%% Kalman Filtering Procedures
close all
% init
P_pri = 0.01 * eye(3);
t = 1;
x_pri = x_init;
x_pos = x_init;
I = eye(3);
figure(1);
grid on
plot_car(x_init);
for i=1:1:10
    url = 'http://192.168.4.1:100';
    options = weboptions('Timeout', 15);
    S=webread(url, options);
    % unpack
    u = [S.tau_L/1000; S.tau_R/1000];
    y = [S.lx; S.ly; x_pos(3)]; % trust state estimation, not the gyro
    % Kalman Gain Update
    [H, C] = hmatrix(x_pos, Lx, Ly);
    K = P_pri * H.' * inv(H * P_pri * H.' + R)
    P_pos = (I - K*H)*P_pri;
    P_pri = P_pos + Q;
    % Measurment and State Estimation Update
    b = bfunction(x_pri, u, Cv, Cr);
    x_pri = x_pos + b;
    [H, C] = hmatrix(x_pri, Lx, Ly);
    x_pos = x_pri + K*(y - H*x_pri - C)
    plot_car(x_pos);
end