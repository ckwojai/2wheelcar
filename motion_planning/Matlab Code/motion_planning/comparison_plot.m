
function output=comparison_plot(x)

Globals;

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
