function IsThisParkingPossible(distance) 
%This function will check if the automatic parking is possible for the
%current parameter settings.
%It will throw error messages if the parking is not possible
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
msg = 'The length of the right obstacle is too small...Change the dimensions and try it again!';
error(msg);


elseif(safe_distance<car_semidiagonal)
%This will check if the parking space is big enough to park, this algorithm
%can successfully park as long as the space's length is bigger than 1.3
%times of the car length.
msg = 'Even for me, this parking space is too narrow... Change the dimensions and try it again!';
error(msg);
    

end
end
