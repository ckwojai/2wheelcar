clear all; close all; clc;
%% Initialization
% Constants in cm
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
%% Simulation Cases
% initialization
x_init = [140; 30; 0];
%%%%%%%%%%%%% Inputs %%%%%%%%%%%%%
u{1}=[1;1]; %This is the first input, it means that "go forward by 120mm".
u{2}=[-0.5, 0.5];%This is the second input, it means that "turn to the left by 30 degree".
u{3}=[1.3;1.3];
u{4}=[0.5;-0.5];
u{5}=[-0.3;-0.3];
u{6}=[0.65;-0.65];
u{7}=[-1, -1];
u{8}=[-0.65;0.65];
u{9}=[-1, -1];
%%%%%%%%%%%%%% Raw Measurements %%%%%%%%%%%%%%%%
%These are the made up measurements for the simulation test, we will need
%to replace them with the real raw measurements from the sensors later.
[x1, y1] = state_evo_to_sensor(x_init, u{1}, Cv, Cr, Lx, Ly);
[x2, y2] = state_evo_to_sensor(x1, u{2}, Cv, Cr, Lx, Ly);
[x3, y3] = state_evo_to_sensor(x2, u{3}, Cv, Cr, Lx, Ly);
[x4, y4] = state_evo_to_sensor(x3, u{4}, Cv, Cr, Lx, Ly);
[x5, y5] = state_evo_to_sensor(x4, u{5}, Cv, Cr, Lx, Ly);
[x6, y6] = state_evo_to_sensor(x5, u{6}, Cv, Cr, Lx, Ly);
[x7, y7] = state_evo_to_sensor(x6, u{7}, Cv, Cr, Lx, Ly);
[x8, y8] = state_evo_to_sensor(x7, u{8}, Cv, Cr, Lx, Ly);
[x9, y9] = state_evo_to_sensor(x8, u{9}, Cv, Cr, Lx, Ly);
m{1}=y1;
m{2}=y2;
m{3}=y3;
m{4}=y4;
m{5}=y5;
m{6}=y6;
m{7}=y7;
m{8}=y8;
m{9}=y9;
figure(1);
subplot(2,1,1);
plot_car(x_init);
hold on
plot_car(x1);
plot_car(x2);
plot_car(x3);
plot_car(x4);
plot_car(x5);
plot_car(x6);
plot_car(x7);
plot_car(x8);
plot_car(x9);
%% Kalman Filter
% init
P_pri = eye(3);
t = 1;
x_pri = x_init;
x_pos = x_init;
I = eye(3);
subplot(2,1,2);
plot_car(x_init);
for i=1:1:9
    % Kalman Gain Update
    [H, C] = hmatrix(x_pri, Lx, Ly);
    K = P_pri * H.' * inv(H * P_pri * H.' + R)
    P_pos = (I - K*H)*P_pri;
    P_pri = P_pos + Q;
    % Measurment and State Estimation Update
    b = bfunction(x_pri, u{i}, Cv, Cr);
    x_pri = x_pos + b;
    [H, C] = hmatrix(x_pri, Lx, Ly);
    x_pos = x_pri + K*(m{i} - H*x_pri - C);
    hold on
    plot_car(x_pos);
end
%% Test: Sensor Measurement
% All test cases matches the non-linear model
x = [Lx/2+100; Ly/2+100; -pi/3];
[H, C] = hmatrix(x, Lx, Ly);
y_act = hfunction(x, Lx, Ly)
y_lin = H*x + C
%% Test: State Evolution
% Rotation centering works ^_^
x_init = [200; 100; 0];
u = [0.5; -0.5];
b = bfunction(x_init, u, Cv, Cr);
x_next = x_init + b
plot_car(x_init);
hold on
plot_car(x_next);
%% Test: Kalmen Filtering with 10 steps
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
    S=webread(url, options)
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
    x_pos = x_pri + K*(y - H*x_pri - C);
    plot_car(x_pos);
end
%% manual
    % Iterative procedure
    url = 'http://192.168.4.1:100';
    options = weboptions('Timeout', 30);
    S=webread(url, options)
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
    x_pos = x_pri + K*(y - H*x_pri - C);
    plot_car(x_pos);