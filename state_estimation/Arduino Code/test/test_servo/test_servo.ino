#include <Servo.h>

Servo servo_left;
Servo servo_right;
int stop_position=90; //For the 360 degree servo, 0 goes full speed clockwise; 90 stop spinning ;180 goes full speed counterclockwise;


//Control Both Wheels
void moving_forward(int velocity,int duration)
{
  servo_left.write(stop_position+velocity);
  servo_right.write(stop_position-velocity-20);
  delay(duration);
  }
  
void moving_backward(int velocity,int duration)
{
  servo_left.write(stop_position-velocity);
  servo_right.write(stop_position+velocity);
  delay(duration);
  }

 void stop_the_car(int duration)
{
  servo_left.write(stop_position);
  servo_right.write(stop_position);
  delay(duration);
  }


//Control a Single Wheel
void moving_forward_right_wheel(int velocity)
{
  servo_right.write(stop_position-velocity);
  
  }
void moving_forward_left_wheel(int velocity)
{
  servo_left.write(stop_position+velocity);
  
  }
void moving_backward_right_wheel(int velocity)
{
  servo_right.write(stop_position+velocity);
 
  }
 void moving_backward_left_wheel(int velocity)
{
  servo_left.write(stop_position-velocity);
  
  }
  void stop_right_wheel()
{
  servo_right.write(stop_position);

  }
   void stop_left_wheel()
{
  servo_left.write(stop_position);
  
  }


void setup() {
servo_left.attach(D1);
servo_right.attach(D2);
}


void loop() {
servo_left.write(0);
servo_right.write(180);
delay(5000);
servo_left.write(90);
servo_right.write(90);
delay(5000);
}
