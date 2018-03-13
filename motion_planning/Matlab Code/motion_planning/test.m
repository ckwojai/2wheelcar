clear all; close all; clc;
global Close_Enough;
global car_length; 
global car_width;
global figure_index;
figure_index=0;
counter=0;

%Get the milestones according to the current dimension settings
Milestones=Get_Milestones;


state_difference=[car_length,-car_width/2,car_length,-car_width/2,car_length,-car_width/2,car_length,-car_width/2];
sensor_states_AtMilestones=Milestones+state_difference
N=length(Milestones)/2+1;







%//////////////////////// Action Required!!! //////////////////////////////
Estimated_State=[160; 80; 0];%Replace this with the real initial reading
%//////////////////////////////////////////////////////////////////////////


for i=1:1:N
Close_Enough=0;

if counter==1
    Estimated_State=[288; 144; 0];

end
if counter==2
    Estimated_State=[425; 143; 0];

end
if counter==3
    Estimated_State=[277; 270; 0];

end
if counter==4
    Estimated_State=[272; 270; 0];
  
end


while Close_Enough==0
[u_turn,u_move,u_extra_turn]=Get_Control_Inputs(Estimated_State,i,Milestones)
figure_index=figure_index+1;

if Close_Enough==0
%//////////////////////// Action Required!!! //////////////////////////////
Estimated_State=[308; 144; 0];%Replace this with the real initial reading
%//////////////////////////////////////////////////////////////////////////
end
end
      counter=counter+1;  
end




