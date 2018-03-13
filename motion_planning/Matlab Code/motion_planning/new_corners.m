function [corner1,corner2,corner3,corner4] = new_corners(x)
% Given current state, find the x,y coordinates of the 
% four corners of the car.
% Start with the sensors corner, rotate anticlockwise 1, 2, 3 ,4
global car_length;
global car_width;
    % Unpack
    r_x = x(1);
    r_y = x(2);
    theta = x(3);
    s = sin(theta);
    c = cos(theta);
    corner1 = [r_x, r_y];
    corner2 = corner1 + [-car_width*s, car_width*c];
    corner3 = corner1 + [-car_length*c - car_width*s, -car_length*s+car_width*c];
    corner4 = corner1 + [-car_length*c, -car_length*s];
end
