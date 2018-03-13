function [x_next, y] = state_evo_to_sensor(x_before, u, Cv, Cr, Lx, Ly)
%   State Evolution, may want to add noise for simulation purposes
    b = bfunction(x_before, u, Cv, Cr);
    x_next = x_before + b;
    y = hfunction(x_next, Lx, Ly) + [10;10;10*pi/180].*wgn(3,1,1);
end

