close all; clc;
% ================================================================================
% Configure the file Globals.m for your application
% In particular, dimensions of environment, obstacle, and car
% Moreover, the coefficient Cv and Cr
% ================================================================================
global Close_Enough;
global figure_index;
figure_index=0;
%Get the milestones according to the current dimension settings
Milestones=Get_Milestones;
N=length(Milestones)/2+1;

% Kalman Init
R = 0.1* eye(3); %measurement noise
Q = zeros(3); %process noise
P_pri = 0.01 * eye(3);
t = 1;
x_pri = x_init;
x_pos = x_init;
I = eye(3);

for i=1:1:N
Close_Enough=0;
Estimated_State=x_pos;%Replace this with the real initial reading.

while Close_Enough==0
[u_turn,u_move,u_extra_turn]=Get_Control_Inputs(Estimated_State,i,Milestones);
% unpack, turn seconds to ms
L1 = round(u_turn(1)*1000);
R1 = round(u_turn(2)*1000);
L2 = round(u_move(1)*1000);
R2 = round(u_move(2)*1000);
L3 = round(u_extra_turn(1)*1000);
R3 = round(u_extra_turn(2)*1000);
%Send u_turn,u_move,u_extra_turn to the car.
options = weboptions('Timeout', 15);
url = 'http://192.168.4.1:100/';
url = strcat(url, num2str(L1), ",", num2str(R1), ",", num2str(L2), ",", num2str(R2), ",", num2str(L3), ",", num2str(R3))
S=webread(url, options);
%Wait util the motion is done.
% Get measurement back
url = 'http://192.168.4.1:100';
S=webread(url, options)
% Kalman Procedure
% unpack
y = [S.lx; S.ly; x_pos(3)] % trust state estimation, not the gyro
% Kalman Gain Update
[H, C] = hmatrix(x_pos, box_length, box_width);
K = P_pri * H.' * inv(H * P_pri * H.' + R);
P_pos = (I - K*H)*P_pri;
P_pri = P_pos + Q;
% Measurment and State Estimation Update
W = car_semidiagonal; % robot diagonal 14cm
% 3 moves
b = bfunction(x_pri, u_turn, Cv, Cr, W);
x_pri = x_pos + b;
b = bfunction(x_pri, u_move, Cv, Cr, W);
x_pri = x_pri + b;
b = bfunction(x_pri, u_extra_turn, Cv, Cr, W);
x_pri = x_pri + b;
[H, C] = hmatrix(x_pri, box_length, box_width);
x_pos = x_pri + K*(y - H*x_pri - C)
figure_index=figure_index+1;
if Close_Enough==0   

Estimated_State=x_pos;%Replace this with the real estimation reading.

end
end 
end



