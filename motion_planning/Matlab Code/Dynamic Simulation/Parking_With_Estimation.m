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


                      
                      
%//////////////////////////////////////////////////////////////////////////
%//////////////////////////////////////////////////////////////////////////
%//////////////////////////////////////////////////////////////////////////





%//////////////////////// Action Required!!! //////////////////////////////
%////////////// Replace This With The Real Life Estimation ////////////////
Read_Estimated_State=[160; 80; 0]; %Estimated initial state
%//////////////////////////////////////////////////////////////////////////




x_ini = Read_Estimated_State; %Store the first estimated state





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
    
   milestone_point{1}=[check_point(1);check_point(2)];%Store the milestone points
   milestone_point{2}=[start(1);start(2)];
   milestone_point{3}=[middle(1);middle(2)];
   milestone_point{4}=[destination(1);destination(2)];
   

   
   
%//////////////////////// The Pre-Parking Process /////////////////////////      
 
     pause on;
     new_plot_car(x_ini);%display the process
     pause(2);
   
 
%This line will recalculate the needed inputs from the current estimated
%state to the next milestone point.
[u_turn1,u_move1]=Get_Inputs(Read_Estimated_State,milestone_point{1},1);


       
%//////////////////////// Action Required!!! //////////////////////////////
%send u_turn1 to the car and wait until this motion is done
%//////////////////////////////////////////////////////////////////////////
 


       x_pre1 = new_state_evo_to_sensor(x_ini, u_turn1);
       trajectory_display(x_ini,x_pre1);%display the process
       new_plot_car(x_pre1);
       pause(1);
 
       
       
%//////////////////////// Action Required!!! //////////////////////////////
%send u_move1 to the car and wait until this motion is done
%//////////////////////////////////////////////////////////////////////////
 


       x_pre2 = new_state_evo_to_sensor(x_pre1, u_move1);
       trajectory_display(x_pre1,x_pre2);%display the process
       new_plot_car(x_pre2);
       pause(1);
 
       
       
%//////////////////////// Action Required!!! //////////////////////////////
%///////////////////// Update Read_Estimated_State ////////////////////////
Read_Estimated_State=[260; 200; 0.39];%Replace this with the real one
%If there are no errors at all, P=0 and Q=0, like in the simulation world,
%it should read this value Read_Estimated_State=[316; 192; 0.4];
%//////////////////////////////////////////////////////////////////////////
       
      
       %Collect states info for future use
       Ideal_State{1}=x_pre2;
       Estimated_State{1}=Read_Estimated_State;
      
      
       x_pre2=Read_Estimated_State;
       new_plot_car(x_pre2);%display the process
       pause(1);
       
       
  
%This line will recalculate the needed inputs from the current estimated
%state to the next milestone point.     
 [u_turn2,u_move2]=Get_Inputs(Read_Estimated_State,milestone_point{2},2);
 
   
 
 
   
%//////////////////////// Action Required!!! //////////////////////////////
%send u_turn2 to the car and wait until this motion is done
%//////////////////////////////////////////////////////////////////////////
 


       x_pre3 = new_state_evo_to_sensor(x_pre2, u_turn2);
       trajectory_display(x_pre2,x_pre3);%display the process
       new_plot_car(x_pre3);
       pause(1);
   
       
       
%//////////////////////// Action Required!!! //////////////////////////////
%send u_move2 to the car and wait until this motion is done
%//////////////////////////////////////////////////////////////////////////
   



       x_pre4 = new_state_evo_to_sensor(x_pre3, u_move2);
       trajectory_display(x_pre3,x_pre4);%display the process
       new_plot_car(x_pre4);
       pause(1);
       

             
       
%//////////////////////// Action Required!!! //////////////////////////////
%///////////////////// Update Read_Estimated_State ////////////////////////
Read_Estimated_State=[410; 150; 0.1]; %Replace this with the real one
%If there are no errors at all, P=0 and Q=0, like in the simulation world,
%it should read this value Read_Estimated_State=[425; 144; 0];
%//////////////////////////////////////////////////////////////////////////
  
       %Collect states info for future use
       Ideal_State{2}=x_pre4;
       Estimated_State{2}=Read_Estimated_State;      
       
       
       


       
       
       
       
%//////////////////////// The Post-Parking Process ////////////////////////   
       
        x_joint=Read_Estimated_State;
        new_plot_car(x_joint);%display the process
        pause(1);
 
         
        
%This line will recalculate the needed inputs from the current estimated
%state to the next milestone point.    
  [u_turn3,u_move3]=Get_Inputs(Read_Estimated_State,milestone_point{3},3);
       
  
  
  
%//////////////////////// Action Required!!! //////////////////////////////
%send u_turn3 to the car and wait until this motion is done
%//////////////////////////////////////////////////////////////////////////




       x_post1 = new_state_evo_to_sensor(x_joint, u_turn3);
       trajectory_display(x_joint,x_post1);%display the process
       new_plot_car(x_post1);
       pause(1);

       
       
       
%//////////////////////// Action Required!!! //////////////////////////////
%send u_move3 to the car and wait until this motion is done
%//////////////////////////////////////////////////////////////////////////
  



       x_post2 = new_state_evo_to_sensor(x_post1, u_move3);
       trajectory_display(x_post1,x_post2);%display the process
       new_plot_car(x_post2);
       pause(1);
       
       
       
%//////////////////////// Action Required!!! //////////////////////////////
%///////////////////// Update Read_Estimated_State ////////////////////////
Read_Estimated_State=[228; 175; -0.67];%Replace this with the real one
%If there are no errors at all, P=0 and Q=0, like in the simulation world,
%it should read this value Read_Estimated_State=[222; 205; -0.7];
%//////////////////////////////////////////////////////////////////////////
       
        %Collect states info for future use
        Ideal_State{3}=x_post2;
        Estimated_State{3}=Read_Estimated_State; 

        x_post2=Read_Estimated_State;
        new_plot_car(x_post2);%display the process
        pause(1);
  
        
        
 
%This line will recalculate the needed inputs from the current estimated
%state to the next milestone point.
  [u_turn4,u_move4]=Get_Inputs(Read_Estimated_State,milestone_point{4},4);
 
  
  
  
  
  
%//////////////////////// Action Required!!! //////////////////////////////
%send u_turn4 to the car and wait until this motion is done
%//////////////////////////////////////////////////////////////////////////
  


       x_post3 = new_state_evo_to_sensor(x_post2, u_turn4);
       trajectory_display(x_post2,x_post3);%display the process
       new_plot_car(x_post3);
       pause(1);
 
       
       
%//////////////////////// Action Required!!! //////////////////////////////
%send u_move4 to the car and wait until this motion is done
%//////////////////////////////////////////////////////////////////////////
     


       x_post4 = new_state_evo_to_sensor(x_post3, u_move4);
       trajectory_display(x_post3,x_post4);%display the process
       new_plot_car(x_post4);
       pause(1);
       
    
       
       
%//////////////////////// Action Required!!! //////////////////////////////
%///////////////////// Update Read_Estimated_State ////////////////////////
Read_Estimated_State=[265; 230; -0.1]; %Replace this with the real one
%If there are no errors at all, P=0 and Q=0, like in the simulation world,
%it should read this value Read_Estimated_State=[273; 270; 0];
%//////////////////////////////////////////////////////////////////////////
       
        %Collect states info for future use
        Ideal_State{4}=x_post4;
        Estimated_State{4}=Read_Estimated_State; 

        x_final=Read_Estimated_State;
        new_plot_car(x_final);%display the result
        pause off;  
        
        
        for i=1:1:4
            figure(i)
            graph1=output_plot(Ideal_State{i});
            hold on;
            graph2=output_plot(Estimated_State{i}) ; 
            hold off;
            legend([graph1 graph2],{'Ideal State','Estimated State'},'Location','southwest');
        end
        
end


%//////////////////////////////////////////////////////////////////////////
%//////////////////////    Parking Completed!     /////////////////////////
%//////////////////////////////////////////////////////////////////////////









%//////////////////////////////////////////////////////////////////////////
%////////////// There's no need to edit anything below ////////////////////
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


function output=output_plot(x)
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
    output=plot(x,y);
    axis([0 box_length 0 box_width]);     
    hold on;
    plot(cor1(1),cor1(2),'o');
    plot(obs1_x,obs1_y,'black','LineWidth',2);
    plot(obs2_x,obs2_y,'black','LineWidth',2);
    hold off;
    
end

function x_next = new_state_evo_to_sensor(x_before, u)
%   State Evolution, may want to add noise for simulation purposes

    b = new_bfunction(x_before, u);
    x_next = x_before + b;
  
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
        x_temp = new_state_evo_to_sensor(x_temp, u);
                new_plot_car(x_temp);%This is one frame of the dynamic graph
                pause(abs(t/N));
        end
else%The car is doing a rotational motion between these to steps
    t=c_difference(3)/Cr;
    for i=1:1:N
        u=[-t/N;t/N]; %Input format for the rotational motion       
        x_temp = new_state_evo_to_sensor(x_temp, u);
                new_plot_car(x_temp);
                pause(abs(t/N));
        
    end
end
end

function [u_turn,u_move]=Get_Inputs(estimated_state,next_point,motion_index)
global Cv; 
global Cr;
estimated_state_center=estimated_state-center2sensor(estimated_state(3));
x_difference=next_point(1)-estimated_state_center(1);
y_difference=next_point(2)-estimated_state_center(2);
angle=atan2(y_difference,x_difference);
helper_sign=1;

moving_input_sign=sign([cos(angle);sin(angle)]'*[x_difference;y_difference]);
turning_angle=(angle-estimated_state(3))/Cr;
if motion_index==3
  turning_angle=((angle-estimated_state(3))-pi)/Cr; 
  helper_sign=-1;
end
if motion_index==4
  turning_angle=-estimated_state(3)/Cr; 
  helper_sign=-1;
end
moving_distance=helper_sign*moving_input_sign*(sqrt((x_difference)^2+(y_difference)^2))/Cv;
u_turn=[-turning_angle;turning_angle];
u_move=[moving_distance;moving_distance];

end

%//////////////////////////////////////////////////////////////////////////
%//////////////////////////////////////////////////////////////////////////
%//////////////////////////////////////////////////////////////////////////