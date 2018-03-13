function Milestones=Get_Milestones

Globals;

half_car_width=car_width/2;

%Half of the parking space length
half_empty_length=(box_length-left_obstacle_length-right_obstacle_length-car_length)/2;

%Destination is the final point that the rotating center of the car must
%reach in the end of the parking.
destination_x=left_obstacle_length+half_empty_length;
destination_y=box_width-safety_distance-half_car_width;


%////////////////// The Core Statements of The Algorithm //////////////////
%It's hard to explain what exactly does these parameters represent without using a
%graph. They are parameters that are used to express the geometry situation
%of the parking space and the starting position.
syms x;
d=box_length-right_obstacle_length-safety_distance;
e=box_width-right_obstacle_width-safety_distance;
c=destination_y;
f=left_obstacle_length+safety_distance;
R=half_car_width;

%This equation expresses the optimal backward parking path
eqn = ((x-f)/R)*sqrt((x-d)^2+(c-e)^2-R^2)-R*sqrt(1-((x-f)/R)^2) == c-e;

%By solving the equation, we can get the best x coordinate of the middle
%potint the car needs to reach before it starts rotating inside the parking
%space.
solx = vpa(solve(eqn, x));
%/////////////////////////////////////////////////////////////////////////


%Get the coordinates of the middle point from the solution
middle_x=double(solx(6));
middle_y=destination_y;

angle1=acos((middle_x-f)/R);
angle2=pi/2-angle1;

%Get the coordinates of the starting point of the backward parking
start_x=box_length-right_obstacle_length;
start_y=middle_y-tan(angle2)*(start_x-middle_x);


%Store the Milestone coordinates
important_distance=sqrt((car_semidiagonal)^2-(box_width-right_obstacle_width-safety_distance-start_y)^2);
Milestones(1)=box_length-right_obstacle_length-safety_distance-important_distance;
Milestones(2)=start_y;
Milestones(3)=start_x;
Milestones(4)=start_y;
Milestones(5)=middle_x;
Milestones(6)=middle_y;
Milestones(7)=destination_x;
Milestones(8)=destination_y;

%Check the dimension settings to determine if it's possible to park
%It will throw error messages if the parking is not possible
IsThisParkingPossible(middle_x);

end
