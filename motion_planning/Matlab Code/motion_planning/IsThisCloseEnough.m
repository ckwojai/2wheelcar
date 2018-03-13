function bool=IsThisCloseEnough(x1,y1,x2,y2,distance)
%This function can determine if the distance between point1 and point2 is
%smaller or equal to the last argument "distance". If the answer is YES, it
%will return 1, otherwise return 0.
d=sqrt((x2-x1)^2+(y2-y1)^2);
if d>distance
    bool=0;
else
    bool=1;
end
end