clear all; close all; clc;

%//////////////////////////////////////////////////////////////////////////
%////////////////////////////Initialization ///////////////////////////////
%//////////////////////////////////////////////////////////////////////////


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

% Constants in mm or radians
Cv = 120;
Cr = 70 * pi/180;
box_length=470;
box_width=370;
left_obstacle_length=120;
left_obstacle_width=100;
right_obstacle_length=160; %For now, don't make this value too small, make it bigger than car_semidiagonal+safety_distance
right_obstacle_width=100;
car_length=115;
car_width=85;
car_semidiagonal=sqrt(car_length^2+(car_width/2)^2);
safety_distance=15; 
N=30;

x_ini = [160; 80; 0]; %The initial state of the sensor of the robot,
                      %NOT the state of its rotating center!
                      %x_ini=[x_coordinate, y_coordinate, heading angle]
                      
%//////////////////////////////////////////////////////////////////////////
%//////////////////////////////////////////////////////////////////////////
%//////////////////////////////////////////////////////////////////////////








%//////////////////////////////////////////////////////////////////////////
%////////////////// Automatic Parallel Parking Algorithm //////////////////
%//////////////////////////////////////////////////////////////////////////
%This Algorithm divides the entire parallel parking problem into two parts,
%namely "The Pre-Parking Process" and "The Post-Parking Process".
%In the pre-parking process, the robot will try to move from an arbitary initial
%position to a position that is as close to the right top obstacle as
%possible without touching any of the walls and the obstacles.
%In the post-parking process, the robot will start from the last
%state of the pre-parking process and try to move into the parking space
%using backward parking without touching anything.

c_ini=x_ini-center2sensor(x_ini(3));%Transpose the sensor state to the center state
[start,middle,destination,inputs] = milestones; %Get the milestone points for the post-parking plan
[situation,check_point]=initial_situation(c_ini(1),c_ini(2),start(2));%Check the initial position
Is_there_a_solution=parking_solution_exists(middle(1)); %Check if it's possible to park in this situation


if (Is_there_a_solution==0||Is_there_a_solution==2)
    fprintf('Sorry, I cannot park in this situation!!!\n'); 

else    
    
 
%//////////////////////// The Pre-Parking Process /////////////////////////      
  %Define important parameters
  angle1=slope_angle(c_ini(1),c_ini(2),check_point(1),check_point(2));       
  turning_angle1=angle1-c_ini(3);
  distance1=distance_between(c_ini(1),c_ini(2),check_point(1),check_point(2));              
  angle2=slope_angle(start(1),start(2),check_point(1),check_point(2));
  turning_angle2=-angle1;
  distance2=distance_between(check_point(1),check_point(2),start(1),start(2));      
  
        
  %Define inputs
  pre_parking_s0_input_1=turning_angle1/Cr;
  pre_parking_s0_input_2=distance1/Cv;
  pre_parking_s0_input_3=turning_angle2/Cr;
  pre_parking_s0_input_4=distance2/Cv;   
  u_s0{1}=[-pre_parking_s0_input_1;pre_parking_s0_input_1]; 
  u_s0{2}=[pre_parking_s0_input_2,pre_parking_s0_input_2];
  u_s0{3}=[-pre_parking_s0_input_3;pre_parking_s0_input_3];
  u_s0{4}=[pre_parking_s0_input_4,pre_parking_s0_input_4];
 
        
  %Get the state of each milestone point on the path
  [x_pre1, y_pre1] = new_state_evo_to_sensor(x_ini, u_s0{1});
  [x_pre2, y_pre2] = new_state_evo_to_sensor(x_pre1, u_s0{2});
  [x_pre3, y_pre3] = new_state_evo_to_sensor(x_pre2, u_s0{3});
  [x_pre4, y_pre4] = new_state_evo_to_sensor(x_pre3, u_s0{4});
  
        
  %Displaying the pre-parking simulation for situation0
  pause on;
  new_plot_car(x_ini);
  pause(2);
  trajectory_display(x_ini,x_pre1); 
  new_plot_car(x_pre1);
  pause(1);
  trajectory_display(x_pre1,x_pre2);
  new_plot_car(x_pre2);
  pause(1);
  trajectory_display(x_pre2,x_pre3);
  new_plot_car(x_pre3);
  pause(1);
  trajectory_display(x_pre3,x_pre4);
  new_plot_car(x_pre4);
  pause(1);

  
   %  x_pre2   
   %  x_pre4
  
    
    
    
%//////////////////////// The Post-Parking Process ////////////////////////
       %Define important parameters
       center_start = [start(1); start(2); 0];
       x_start=center_start+center2sensor(center_start(3));
       
       %Define inputs
       input_index_1=inputs(1);
       input_index_2=inputs(2);
       input_index_3=-inputs(1);
       input_index_4=inputs(3);
       u{1}=[-input_index_1;input_index_1]; 
       u{2}=[input_index_2,input_index_2];
       u{3}=[-input_index_3;input_index_3];
       u{4}=[input_index_4;input_index_4];

       %Get the state of each milestone point on the path
       [x_post1, y_post1] = new_state_evo_to_sensor(x_start, u{1});
       [x_post2, y_post2] = new_state_evo_to_sensor(x_post1, u{2});
       [x_post3, y_post3] = new_state_evo_to_sensor(x_post2, u{3});
       [x_post4, y_post4] = new_state_evo_to_sensor(x_post3, u{4});

       %Displaying the post-parking simulation
       trajectory_display(x_pre4,x_post1); 
       new_plot_car(x_post1);
       pause(1);
       trajectory_display(x_post1,x_post2);
       new_plot_car(x_post2);
       pause(1);
      
       trajectory_display(x_post2,x_post3);
       new_plot_car(x_post3);
       pause(1);
       trajectory_display(x_post3,x_post4);
       new_plot_car(x_post4);   
       
       pause off;
       
      %  x_post2
      %  x_post4

end

%//////////////////////////////////////////////////////////////////////////
%//////////////////////////////////////////////////////////////////////////
%//////////////////////////////////////////////////////////////////////////








%//////////////////////////////////////////////////////////////////////////
%//////////////////////////// Functions  //////////////////////////////////
%//////////////////////////////////////////////////////////////////////////


function new_plot_car(x)
%UNTITLED10 Summary of this function goes here
%   Detailed explanation goes here
global box_length;
global box_width;
global left_obstacle_length;
global left_obstacle_width;
global right_obstacle_length;
global right_obstacle_width;
obs1_x=[0;left_obstacle_length;left_obstacle_length;0;0];
obs1_y=[box_width;box_width;box_width-left_obstacle_width;box_width-left_obstacle_width;box_width];
obs2_x=[box_length-right_obstacle_length;box_length;box_length;box_length-right_obstacle_length;box_length-right_obstacle_length];
obs2_y=[box_width;box_width;box_width-right_obstacle_width;box_width-right_obstacle_width;box_width];
    [cor1, cor2, cor3, cor4] = new_corners(x);
    x=[cor1(1);cor2(1);cor3(1);cor4(1);cor1(1)];
    y=[cor1(2);cor2(2);cor3(2);cor4(2);cor1(2)];
    plot(x,y);
    axis([0 box_length 0 box_width]);     
    hold on;
    plot(cor1(1),cor1(2),'o');
    plot(obs1_x,obs1_y,'black','LineWidth',2);
    plot(obs2_x,obs2_y,'black','LineWidth',2);
    hold off;
    
end

function [x_next, y] = new_state_evo_to_sensor(x_before, u)
%   State Evolution, may want to add noise for simulation purposes
global box_length;
global box_width;
    b = new_bfunction(x_before, u);
    x_next = x_before + b;
   % y = hfunction(x_next, box_length, box_width) + [10;10;10*pi/180].*wgn(3,1,1);
   y=11;
end

function [corner1,corner2,corner3,corner4] = new_corners(x)
% Given current state, find the x,y coordinates of the 
% four corners of the car.
% Start with the sensors corner, rotate anticlockwise 1, 2, 3 ,4
    % Constant
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

function [ output ] = new_bfunction(x, u)
% Constant
global Cv;
global Cr;
global car_length;
global car_width;
global car_semidiagonal; 
    angle_offset = atan((car_width/2)/car_length);
    theta = x(3);
    tau_L = u(1);
    tau_R = u(2);
    p_str = (tau_L * tau_R) > 0;
    c = cos(theta);
    s = sin(theta);
    a1 = theta - angle_offset;
    a2 = theta + Cr*tau_R - angle_offset;
    a = p_str*Cv*c*tau_R-(1-p_str)*car_semidiagonal*(cos(a1)-cos(a2));
    d = p_str*Cv*s*tau_R+(1-p_str)*car_semidiagonal*(sin(a2)-sin(a1));
    c = (1-p_str)*Cr*tau_R;
    output = [a;d;c];  
end

%//////////////////////////////////////////////////////////////////////////
%//////////////////////////////////////////////////////////////////////////
%//////////////////////////////////////////////////////////////////////////








%//////////////////////////////////////////////////////////////////////////
%/////////////////////////// New Functions ////////////////////////////////
%//////////////////////////////////////////////////////////////////////////


function state_difference=center2sensor(current_angle)
%This function gives the state difference between the rotating center and
%the sensor corner. You can get sensor_state=center_state+state_difference,
%you can also get center_state=sensor_state-state_difference.
global car_length;
global car_width;
s=sin(current_angle);
c=cos(current_angle);
state_difference=[c -s;s c;0 0]*[car_length;-car_width/2];
end

function d=distance_between(x1,y1,x2,y2)
%This function gives the distance between two points, the order of the
%arguments doesn's matter.
d=sqrt((x1-x2)^2+(y1-y2)^2);
end

function a=slope_angle(x_start,y_start,x_end,y_end)
%This function gives the angle of the vector that is defined by a starting
%point and an ending point. So the order of the arguments MATTERS!!!
x=x_end-x_start;
y=y_end-y_start;
a=atan2(y,x);
end

function bool=parking_solution_exists(distance) 
%This function will check if the automatic parking is possible for the
%current situation. If it's possible it will return 1.
global box_length;
global right_obstacle_length;
global safety_distance;
global car_semidiagonal;
safe_distance=box_length-right_obstacle_length-safety_distance-distance;

if right_obstacle_length<(safety_distance+car_semidiagonal)
%This will check the length of the right top obstacle, the right top
%obstacle provides the post parking starting position frame for the current
%version of this algorithm, so at least it has to be approximately as big as the
%car.
    bool=2;
    fprintf('The length of the right obstacle is too small...\n');
elseif(safe_distance<car_semidiagonal)
%This will check if the parking space is big enough to park, this algorithm
%can successfully park as long as the space's length is bigger than 1.3
%times of the car length.
    bool=0;
    fprintf('Even for me, this parking space is too narrow...\n'); 
else
    bool=1;%This means it's possible to park automatically.
end
end

function [start,middle,destination,inputs] = milestones
%It's the most important function of this algorithm.
%This function determines the coordinates of three important locations 
%for the post parking plan and collects the inputs to get to thoses locations    
global Cv;
global Cr;
global box_length;
global box_width;
global left_obstacle_length;
global right_obstacle_length;
global right_obstacle_width;
global car_length;
global car_width;
global safety_distance;
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
middle_x=double(solx(1));
middle_y=destination_y;

angle1=acos((middle_x-f)/R);
angle2=pi/2-angle1;

%Get the coordinates of the starting point of the backward parking
start_x=box_length-right_obstacle_length;
start_y=middle_y-tan(angle2)*(start_x-middle_x);

%Store the inputs that are needed for this motion plan
input1 = -angle2/Cr;
input2=-distance_between(middle_x,middle_y,start_x,start_y)/Cv;
input3=(destination_x-middle_x)/Cv;

%Output the coordinates of the milestone points and the corresponding
%inputs.
start = [start_x, start_y];
middle = [middle_x, middle_y];
destination = [destination_x, destination_y];
inputs=[input1,input2,input3];

end



function [situation,check_point]=initial_situation(x_ini,y_ini,y_milestone)
%This function will check the given initial coordinates of the car and
%return differenct values accordingly. The "situation" value will be used
%for the pre-parking plan(from initial point to the backward parking
%starting point). We will have slightly different pre-parking plans due to the 
%different situations. The "check_point" it kind of the middle point between the
%initial point and the backward parking starting point.
%Remember the arguments of this function is not the coordinates of the
%sensor but the rotating center.

global box_length;
global box_width;
global left_obstacle_length;
global left_obstacle_width;
global right_obstacle_length;
global right_obstacle_width;
global car_semidiagonal;
global safety_distance;

%Define a rectangular boundary around the center of the box
boundary_y_up=box_width-max(left_obstacle_width,right_obstacle_width)-car_semidiagonal-safety_distance;
boundary_y_down=car_semidiagonal+safety_distance;
boundary_x_left=car_semidiagonal+safety_distance;
boundary_x_right=box_length-car_semidiagonal-safety_distance;
parking_space_middleline_x=left_obstacle_length+(box_length-left_obstacle_length-right_obstacle_length)/2;

%Store the check point
important_distance=sqrt((car_semidiagonal)^2-(box_width-right_obstacle_width-safety_distance-y_milestone)^2);
check_x=box_length-right_obstacle_length-safety_distance-important_distance;
check_y=y_milestone;
check_point=[check_x;check_y];


if x_ini>boundary_x_left&&x_ini<boundary_x_right&&y_ini<boundary_y_up&&y_ini>boundary_y_down
    situation=1;%This means the initial position is close to the center of the box
else
    situation=0;%This means the initial position is close to the walls or the obstacles
end

end


function trajectory_display(x1,x2)
%This function is written to display a dynamic simulation graph
global Cv; 
global Cr;
global N;
c1=x1-center2sensor(x1(3));%Transpose the sensor state to the center state
c2=x2-center2sensor(x1(3));
c_difference=c2-c1;
x_temp=x1;

if c_difference(3)==0 %This means the car is doing a translational motion between these to steps
    input_sign=sign([cos(x1(3));sin(x1(3))]'*[c_difference(1);c_difference(2)]);
    t=input_sign*sqrt((c_difference(1))^2+(c_difference(2))^2)/Cv;
        for i=1:1:N %Divide the motion into N pieces
        u=[t/N;t/N]; %Input format for the translational motion       
        [x_temp, y_temp] = new_state_evo_to_sensor(x_temp, u);
                new_plot_car(x_temp);%This is one frame of the dynamic graph
                pause(abs(t/N));
        end
else%The car is doing a rotational motion between these to steps
    t=c_difference(3)/Cr;
    for i=1:1:N
        u=[-t/N;t/N]; %Input format for the rotational motion       
        [x_temp, y_temp] = new_state_evo_to_sensor(x_temp, u);
                new_plot_car(x_temp);
                pause(abs(t/N));
        
    end
end
end

%//////////////////////////////////////////////////////////////////////////
%//////////////////////////////////////////////////////////////////////////
%//////////////////////////////////////////////////////////////////////////