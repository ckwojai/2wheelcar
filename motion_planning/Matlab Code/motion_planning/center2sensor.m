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