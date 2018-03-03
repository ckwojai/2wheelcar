function plot_car(x)
%UNTITLED10 Summary of this function goes here
%   Detailed explanation goes here
    [cor1, cor2, cor3, cor4] = corners(x);
    x=[cor1(1);cor2(1);cor3(1);cor4(1);cor1(1)];
    y=[cor1(2);cor2(2);cor3(2);cor4(2);cor1(2)];
    plot(x,y);
    axis([0 490 0 360]);
    hold on
    plot(cor1(1),cor1(2),'o');
    drawnow;
    hold on
end

