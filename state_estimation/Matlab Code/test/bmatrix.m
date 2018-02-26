function [ B ] = bmatrix(p_str, Cv, Cr, theta, u)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
    B = [0 p_str*Cv*cos(theta); 0 p_str*Cv*sin(theta); 0 (1-p_str)*Cr];
end

