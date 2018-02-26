function [H, C] = hmatrix(x0, l0, Lx, Ly)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
    rx0 = x0(1);
    ry0 = x0(2);
    theta0 = x0(3);
    lx0 = l0(1);
    ly0 = l0(2);
    H = [-1/cos(theta0), 0, (Lx-rx0)*sin(theta0)/(cos(theta0))^2;
        0, 1/cos(theta0), ry0*sin(theta0)/(cos(theta0))^2;
        0, 0, 1
        ];
    C = [rx0/cos(theta0) - (Lx-rx0)*sin(theta0)/(cos(theta0))^2 * theta0 + lx0;
         -ry0/cos(theta0) - ry0*sin(theta0)/(cos(theta0))^2 * theta0 + ly0;
         0
        ];
end

