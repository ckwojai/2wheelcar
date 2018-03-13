function a=slope_angle(x_start,y_start,x_end,y_end)
%This function gives the angle of the vector that is defined by a starting
%point and an ending point. So the order of the arguments MATTERS!!!
x=x_end-x_start;
y=y_end-y_start;
a=atan2(y,x);
end