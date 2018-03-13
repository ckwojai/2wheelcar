function [u_turn,u_move,u_extra_turn]=Get_Control_Inputs(Estimated_State,Estimation_Index,Milestones)

global Cv; %Translational coefficient of the robot
global Cr; %Rotational coefficient of the robot
global acceptable_radius1; 
global acceptable_radius2;
global acceptable_radius3;
global acceptable_radius4;
global Close_Enough;
global figure_index;


estimated_state_center=Estimated_State-center2sensor(Estimated_State(3));
if Estimation_Index~=5
x_difference=Milestones(Estimation_Index*2-1)-estimated_state_center(1);
y_difference=Milestones(Estimation_Index*2)-estimated_state_center(2);
angle=atan2(y_difference,x_difference);

direction=1;%1 means it should move forward, 0 means it should move backward.
if x_difference<0
    direction=0;
end

end

if Estimation_Index==1
    if direction==1
        turning_angle=(angle-Estimated_State(3))/Cr;
        moving_distance=sqrt((x_difference)^2+(y_difference)^2)/Cv;
        extra_turning=-angle/Cr;
        u_turn=[-turning_angle;turning_angle];
        u_move=[moving_distance;moving_distance];
        u_extra_turn=[-extra_turning;extra_turning];
    else
        angle_sign=sign(angle);
        angle=angle-angle_sign*pi;
        turning_angle=(angle-Estimated_State(3))/Cr;
        moving_distance=-sqrt((x_difference)^2+(y_difference)^2)/Cv;
        extra_turning=-angle/Cr;
        u_turn=[-turning_angle;turning_angle];
        u_move=[moving_distance;moving_distance];
        u_extra_turn=[-extra_turning;extra_turning];
    end
Close_Enough=1;
    
elseif Estimation_Index==2
    x_pre=Milestones(Estimation_Index*2-3);
    y_pre=Milestones(Estimation_Index*2-2);
    Close_Enough=IsThisCloseEnough(estimated_state_center(1),estimated_state_center(2),x_pre,y_pre,acceptable_radius1);
    if Close_Enough==0
       x_difference=x_pre-estimated_state_center(1);
       y_difference=y_pre-estimated_state_center(2);
       angle=atan2(y_difference,x_difference);
       direction=1;%1 means it should move forward, 0 means it should move backward.
       if x_difference<0
           direction=0;
       end
    end    
   
           if direction==1
        turning_angle=(angle-Estimated_State(3))/Cr;
        moving_distance=sqrt((x_difference)^2+(y_difference)^2)/Cv;
        extra_turning=-angle/Cr;
        u_turn=[-turning_angle;turning_angle];
        u_move=[moving_distance;moving_distance];
        u_extra_turn=[-extra_turning;extra_turning];
        else
        angle_sign=sign(angle);
        angle=angle-angle_sign*pi;
        turning_angle=(angle-Estimated_State(3))/Cr;
        moving_distance=-sqrt((x_difference)^2+(y_difference)^2)/Cv;
        extra_turning=-angle/Cr;
        u_turn=[-turning_angle;turning_angle];
        u_move=[moving_distance;moving_distance];
        u_extra_turn=[-extra_turning;extra_turning];
           end
          
        Expected_Center_State=[Milestones(Estimation_Index*2-3);Milestones(Estimation_Index*2-2);0];
        Expected_State=Expected_Center_State+center2sensor(Expected_Center_State(3));
        figure(figure_index)
        graph1=comparison_plot(Expected_State);
        hold on;
        graph2=comparison_plot(Estimated_State) ; 
        hold off;
        legend([graph1 graph2],{'Expected State','Estimated State'},'Location','southwest');
   
elseif Estimation_Index==3
    x_pre=Milestones(Estimation_Index*2-3);
    y_pre=Milestones(Estimation_Index*2-2);
    Close_Enough=IsThisCloseEnough(estimated_state_center(1),estimated_state_center(2),x_pre,y_pre,acceptable_radius2);
    if Close_Enough==0
       x_difference=x_pre-estimated_state_center(1);
       y_difference=y_pre-estimated_state_center(2);
       angle=atan2(y_difference,x_difference);
       direction=1;%1 means it should move forward, 0 means it should move backward.
       if x_difference<0
           direction=0;
       end
    end    
   
           if direction==1
        turning_angle=(angle-Estimated_State(3))/Cr;
        moving_distance=sqrt((x_difference)^2+(y_difference)^2)/Cv;
        extra_turning=-angle/Cr;
        u_turn=[-turning_angle;turning_angle];
        u_move=[moving_distance;moving_distance];
        u_extra_turn=[-extra_turning;extra_turning];
        else
        angle_sign=sign(angle);
        angle=angle-angle_sign*pi;
        turning_angle=(angle-Estimated_State(3))/Cr;
        moving_distance=-sqrt((x_difference)^2+(y_difference)^2)/Cv;
        extra_turning=-angle/Cr;
        u_turn=[-turning_angle;turning_angle];
        u_move=[moving_distance;moving_distance];
        u_extra_turn=[-extra_turning;extra_turning];
           end
        Expected_Center_State=[Milestones(Estimation_Index*2-3);Milestones(Estimation_Index*2-2);0];
        Expected_State=Expected_Center_State+center2sensor(Expected_Center_State(3));
        figure(figure_index)
        graph1=comparison_plot(Expected_State);
        hold on;
        graph2=comparison_plot(Estimated_State) ; 
        hold off;
        legend([graph1 graph2],{'Expected State','Estimated State'},'Location','southwest');
        
        
elseif Estimation_Index==4
    x_pre=Milestones(Estimation_Index*2-3);
    y_pre=Milestones(Estimation_Index*2-2);
    Close_Enough=IsThisCloseEnough(estimated_state_center(1),estimated_state_center(2),x_pre,y_pre,acceptable_radius3);
    if Close_Enough==0
       x_difference=x_pre-estimated_state_center(1);
       y_difference=y_pre-estimated_state_center(2);
       angle=atan2(y_difference,x_difference);
       direction=1;%1 means it should move forward, 0 means it should move backward.
       if x_difference<0
           direction=0;
       end
    end    
   
           if direction==1
        turning_angle=(angle-Estimated_State(3))/Cr;
        moving_distance=sqrt((x_difference)^2+(y_difference)^2)/Cv;
        extra_turning=-angle/Cr;
        u_turn=[-turning_angle;turning_angle];
        u_move=[moving_distance;moving_distance];
        u_extra_turn=[-extra_turning;extra_turning];
        else
        angle_sign=sign(angle);
        angle=angle-angle_sign*pi;
        turning_angle=(angle-Estimated_State(3))/Cr;
        moving_distance=-sqrt((x_difference)^2+(y_difference)^2)/Cv;
        extra_turning=-angle/Cr;
        u_turn=[-turning_angle;turning_angle];
        u_move=[moving_distance;moving_distance];
        u_extra_turn=[-extra_turning;extra_turning];
           end
           
        Expected_Center_State=[Milestones(Estimation_Index*2-3);Milestones(Estimation_Index*2-2);0];
        Expected_State=Expected_Center_State+center2sensor(Expected_Center_State(3));
        figure(figure_index)
        graph1=comparison_plot(Expected_State);
        hold on;
        graph2=comparison_plot(Estimated_State) ; 
        hold off;
        legend([graph1 graph2],{'Expected State','Estimated State'},'Location','southwest');
        
        
else
    x_pre=Milestones(Estimation_Index*2-3);
    y_pre=Milestones(Estimation_Index*2-2);

    
    Close_Enough=IsThisCloseEnough(estimated_state_center(1),estimated_state_center(2),x_pre,y_pre,acceptable_radius4);
    if Close_Enough==0
       x_difference=x_pre-estimated_state_center(1);
       y_difference=y_pre-estimated_state_center(2);
       angle=atan2(y_difference,x_difference);
       direction=1;%1 means it should move forward, 0 means it should move backward.
       if x_difference<0
           direction=0;
       end 
           if direction==1
        turning_angle=(angle-Estimated_State(3))/Cr;
        moving_distance=sqrt((x_difference)^2+(y_difference)^2)/Cv;
        extra_turning=-angle/Cr;
        u_turn=[-turning_angle;turning_angle];
        u_move=[moving_distance;moving_distance];
        u_extra_turn=[-extra_turning;extra_turning];
        else
        angle_sign=sign(angle);
        angle=angle-angle_sign*pi;
        turning_angle=(angle-Estimated_State(3))/Cr;
        moving_distance=-sqrt((x_difference)^2+(y_difference)^2)/Cv;
        extra_turning=-angle/Cr;
        u_turn=[-turning_angle;turning_angle];
        u_move=[moving_distance;moving_distance];
        u_extra_turn=[-extra_turning;extra_turning];
           end
    else
        u_turn=[0;0];
        u_move=[0;0];
        u_extra_turn=[0;0];
    
    end 

    
        Expected_Center_State=[Milestones(Estimation_Index*2-3);Milestones(Estimation_Index*2-2);0];
        Expected_State=Expected_Center_State+center2sensor(Expected_Center_State(3));
        figure(figure_index)
        graph1=comparison_plot(Expected_State);
        hold on;
        graph2=comparison_plot(Estimated_State) ; 
        hold off;
        legend([graph1 graph2],{'Expected State','Estimated State'},'Location','southwest');
        
        
end
end

