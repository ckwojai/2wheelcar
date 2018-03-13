%//////////////////////////////////////////////////////////////////////////
%////////////////////////////Initialization ///////////////////////////////
%//////////////////////////////////////////////////////////////////////////

global x_init
x_init = [160; 40; 0];
global Cv; %Translational coefficient of the robot
global Cr; %Rotational coefficient of the robot
global box_length; %The dimension of the box, this is the longer side
global box_width;  %This is the shorter side
global left_obstacle_length; %The distance from left to right of the obstacle located on the left top corner
global left_obstacle_width;  %The distance from up to down of the obstacle located on the left top corner
global right_obstacle_length; %The distance from left to right of the obstacle located on the right top corner
global right_obstacle_width; %The distance from up to down of the obstacle located on the right top corner
global car_length; %The dimensions of the robot
global car_width;
global car_semidiagonal; %The distance from the rotating center to the further corner of the car
global safety_distance; %The distance we want the car to keep from the walls and obstacles
global N; %A constant that is used to control the simulation speed, the smaller N is, the faster the simulation.
global figure_index;


%These are the acceptable radius,
%the car will keep moving until it gets to a point within this range from the target point, 
%then move to the next milestone.
global acceptable_radius1; 
global acceptable_radius2;
global acceptable_radius3;
global acceptable_radius4;
global Close_Enough;%Determine if it's close enough to the target position.

% Constants in mm or radians
% Cv and Cr need to be adjusted
Cv = 180;
Cr = 140 * pi/180;
box_length=485;
box_width=370;
left_obstacle_length=120;
left_obstacle_width=100;
right_obstacle_length=160; %For now, don't make this value too small, make it bigger than car_semidiagonal+safety_distance
right_obstacle_width=100;
car_length=115;
car_width=85;
car_semidiagonal=sqrt(car_length^2+(car_width/2)^2);
safety_distance=20; 
N=30;

%make the value small if you have a higher level accuracy requirement for a specific point.
acceptable_radius1=300;%distance in mm
acceptable_radius2=300;
acceptable_radius3=300;
acceptable_radius4=300;



%//////////////////////////////////////////////////////////////////////////
%//////////////////////////////////////////////////////////////////////////
%//////////////////////////////////////////////////////////////////////////
