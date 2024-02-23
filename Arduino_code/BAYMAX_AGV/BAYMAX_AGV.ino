#include <Wire.h>
#include <PID_v1.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include <std_msgs/Int16.h>


//initializing all the variables
#define LOOPTIME                      100     //Looptime in millisecond
const byte noCommLoopMax = 10;                //number of main loops the robot will execute without communication before stopping
unsigned int noCommLoops = 0;                 //main loop without communication counter

const long int encoder_minimum = -32768;
const long int encoder_maximum = 32767;

double speed_cmd_left2 = 0;

const int left_encoderA = 2;               //A channel for encoder of left motor
const int left_encoderB = 8;      //B channel for encoder of left motor
volatile long left_encoder_value = 0;
volatile int left_wheel_tick_count = 0;
std_msgs::Int16 left_wheel_tick_count_msg;


const int right_encoderA = 3;              //A channel for encoder of right motor
const int right_encoderB = 9; //B channel for encoder of right motor
volatile long right_encoder_value = 0;
volatile int right_wheel_tick_count = 0;
std_msgs::Int16 right_wheel_tick_count_msg;


// Pin variables for motors.
const int right_pwm = 5;
const int right_dir = 4;
const int left_pwm = 7;
const int left_dir = 6;

int buzzer = 10;


unsigned long lastMilli = 0;

//--- Robot-specific constants ---
const double radius = 0.05;                   //Wheel radius, in m
const double wheelbase = 0.26;               //Wheelbase, in m
const double encoder_cpr = 2940;               //Encoder ticks or counts per rotation
const double speed_to_pwm_ratio = 0.00369;    //Ratio to convert speed (in m/s) to PWM value. It was obtained by plotting the wheel speed in relation to the PWM motor command (the value is the slope of the linear function).
//const double min_speed_cmd = 0.20295;          //(min_speed_cmd/speed_to_pwm_ratio) is the minimum command value needed for the motor to start moving. This value was obtained by plotting the wheel speed in relation to the PWM motor command (the value is the constant of the linear function).
const double min_speed_cmd = 0.2214;     // for 60 pwm


double speed_req = 0;                         //Desired linear speed for the robot, in m/s
double angular_speed_req = 0;                 //Desired angular speed for the robot, in rad/s

double speed_req_left = 0;                    //Desired speed for left wheel in m/s
double speed_act_left = 0;                    //Actual speed for left wheel in m/s
double speed_cmd_left = 0;                    //Command speed for left wheel in m/s

double speed_req_right = 0;                   //Desired speed for right wheel in m/s
double speed_act_right = 0;                   //Actual speed for right wheel in m/s
double speed_cmd_right = 0;                   //Command speed for right wheel in m/s

const double max_speed = 1.0;                 //Max speed in m/s

int PWM_leftMotor = 0;                     //PWM command for left motor
int PWM_rightMotor = 0;                    //PWM command for right motor

// PID Parameters
const double PID_left_param[] = { 0.0, 0, 0.0 }; //Respectively Kp, Ki and Kd for left motor PID
const double PID_right_param[] = { 0.0, 0, 0.0 }; //Respectively Kp, Ki and Kd for right motor PID

volatile float pos_left = 0;       //Left motor encoder position
volatile float pos_right = 0;      //Right motor encoder position

PID PID_leftMotor(&speed_act_left, &speed_cmd_left, &speed_req_left, PID_left_param[0], PID_left_param[1], PID_left_param[2], DIRECT);          //Setting up the PID for left motor
PID PID_rightMotor(&speed_act_right, &speed_cmd_right, &speed_req_right, PID_right_param[0], PID_right_param[1], PID_right_param[2], DIRECT);   //Setting up the PID for right motor


ros::NodeHandle nh;

//function that will be called when receiving command from host
void handle_cmd (const geometry_msgs::Twist& cmd_vel) {
  noCommLoops = 0;                                                  //Reset the counter for number of main loops without communication

  speed_req = cmd_vel.linear.x;                                     //Extract the commanded linear speed from the message

  angular_speed_req = cmd_vel.angular.z;                            //Extract the commanded angular speed from the message

  speed_req_left = speed_req - angular_speed_req * (wheelbase / 2); //Calculate the required speed for the left motor to comply with commanded linear and angular speeds
  speed_req_right = speed_req + angular_speed_req * (wheelbase / 2); //Calculate the required speed for the right motor to comply with commanded linear and angular speeds

}

// ros publisher
ros::Publisher left_speed_output("lwheel", &left_wheel_tick_count_msg);
ros::Publisher right_speed_output("rwheel", &right_wheel_tick_count_msg);

ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", handle_cmd);   //create a subscriber to ROS topic for velocity commands (will execute "handle_cmd" function when receiving data)
//geometry_msgs::Vector3Stamped speed_msg;                                //create a "speed_msg" ROS message
//ros::Publisher speed_pub("speed", &speed_msg);                          //create a publisher to ROS topic "speed" using the "speed_msg" type

void left_Fwd(const size_t speed) {
  analogWrite(left_pwm, speed); // done
  digitalWrite(left_dir, 1);
}
void left_Bwd(const size_t speed) {
  analogWrite(left_pwm, speed); //done
  digitalWrite(left_dir, 0);
}
void left_Stop() {
  analogWrite(left_pwm, 0);
  digitalWrite(left_dir, 0);
}

void right_Fwd(const size_t speed) {
  analogWrite(right_pwm, speed);
  digitalWrite(right_dir, 1);
}
void right_Bwd(const size_t speed) {
  analogWrite(right_pwm, speed);
  digitalWrite(right_dir, 0);
}
void    right_Stop() {
  analogWrite(right_pwm, 0);
  digitalWrite(right_dir, 0);
}


//__________________________________________________________________________

void setup() {

  pinMode(right_pwm, OUTPUT);    // sets the digital pin 13 as output
  pinMode(left_pwm, OUTPUT);
  pinMode(right_dir, OUTPUT);
  pinMode(left_dir, OUTPUT);

  playTone(1500, 125);  // First tone
  delay(20);
  playTone(1800, 125);  // Second tone
  delay(20);
  playTone(2100, 125);  // Third tone
  delay(20);
  playTone(2400, 125);  // Fourth tone
  delay(20);
  playTone(2700, 125);  // Fifth tone

  delay(1000);


  nh.initNode();                            //init ROS node
  nh.getHardware()->setBaud(57600);         //set baud for ROS serial communication
  nh.subscribe(cmd_vel);
  nh.advertise(left_speed_output);
  nh.advertise(right_speed_output);//suscribe to ROS topic for velocity commands
  //nh.advertise(speed_pub);                  //prepare to publish speed in ROS topic


  //setting motor speeds to zero
  left_Stop();
  right_Stop();
  //setting PID parameters
  //PID_leftMotor.SetSampleTime(5);
  //PID_rightMotor.SetSampleTime(5);
  PID_leftMotor.SetOutputLimits(-max_speed, max_speed);
  PID_rightMotor.SetOutputLimits(-max_speed, max_speed);
  PID_leftMotor.SetMode(AUTOMATIC);
  PID_rightMotor.SetMode(AUTOMATIC);

  // Define the rotary encoder for left motor
  pinMode(left_encoderA, INPUT);
  pinMode(left_encoderB, INPUT);
  digitalWrite(left_encoderA, HIGH);                // turn on pullup resistor
  digitalWrite(left_encoderB, HIGH);
  attachInterrupt(0, encoderLeftMotor, RISING);

  // Define the rotary encoder for right motor
  pinMode(right_encoderA, INPUT);
  pinMode(right_encoderB, INPUT);
  digitalWrite(right_encoderA, HIGH);                // turn on pullup resistor
  digitalWrite(right_encoderB, HIGH);
  attachInterrupt(1, encoderRightMotor, RISING);
}

//_________________________________________________________________________

void loop() {
  nh.spinOnce();
  if ((millis() - lastMilli) >= LOOPTIME)
  { // enter timed loop
    lastMilli = millis();

    if (abs(pos_left) < 5) {                                                  //Avoid taking in account small disturbances
      speed_act_left = 0;
    }
    else {
      speed_act_left = ((pos_left / encoder_cpr) * 2 * PI) * (1000 / LOOPTIME) * radius; // calculate speed of left wheel
    }

    if (abs(pos_right) < 5) {                                                 //Avoid taking in account small disturbances
      speed_act_right = 0;
    }
    else {
      speed_act_right = ((pos_right / encoder_cpr) * 2 * PI) * (1000 / LOOPTIME) * radius; // calculate speed of right wheel
    }

  
    pos_left = 0;
    pos_right = 0;

    speed_cmd_left = constrain(speed_cmd_left, -max_speed, max_speed);
    PID_leftMotor.Compute();
    // compute PWM value for left motor. Check constant definition comments for more information.
    PWM_leftMotor = constrain(((speed_req_left + sgn(speed_req_left) * min_speed_cmd) / speed_to_pwm_ratio) + (speed_cmd_left / speed_to_pwm_ratio), -255, 255); //

    speed_cmd_right = constrain(speed_cmd_right, -max_speed, max_speed);
    PID_rightMotor.Compute();
    // compute PWM value for right motor. Check constant definition comments for more information.
    PWM_rightMotor = constrain(((speed_req_right + sgn(speed_req_right) * min_speed_cmd) / speed_to_pwm_ratio) + (speed_cmd_right / speed_to_pwm_ratio), -255, 255); //
    

    if (noCommLoops >= noCommLoopMax) {                   //Stopping if too much time without command
      left_Stop();

    }
    else if (speed_req_left == 0) {                       //Stopping
      left_Stop();
    }
    else if (PWM_leftMotor < 0) {
      left_Fwd(abs(PWM_leftMotor));                   //Going forward
    }
    else {
      left_Bwd(abs(PWM_leftMotor));                   //Going backward

    }


    if (noCommLoops >= noCommLoopMax) {                   //Stopping if too much time without command
      right_Stop();
    }
    else if (speed_req_right == 0) {                      //Stopping
      right_Stop();
    }
    else if (PWM_rightMotor < 0) {                        //Going forward
      right_Fwd(abs(PWM_rightMotor));
    }
    else {                                                //Going backward
      right_Bwd(abs(PWM_rightMotor));
    }

    if ((millis() - lastMilli) >= LOOPTIME) {     //write an error if execution time of the loop in longer than the specified looptime
      nh.loginfo(" TOO LONG ");
    }

    noCommLoops++;
    if (noCommLoops == 65535) {
      noCommLoops = noCommLoopMax;
    }

    //publishSpeed(LOOPTIME);   //Publish odometry on ROS topic
  }
  right_wheel_tick_count_msg.data = right_wheel_tick_count;
  left_wheel_tick_count_msg.data = left_wheel_tick_count;

  right_speed_output.publish(&right_wheel_tick_count_msg);
  left_speed_output.publish(&left_wheel_tick_count_msg);
}

//Publish function for odometry, uses a vector type message to send the data (message type is not meant for that but that's easier than creating a specific message type)
/*void publishSpeed(double time) {
  speed_msg.header.stamp = nh.now();      //timestamp for odometry data
  speed_msg.vector.x = speed_act_left;    //left wheel speed (in m/s)
  speed_msg.vector.y = speed_act_right;   //right wheel speed (in m/s)
  speed_msg.vector.z = time / 1000;       //looptime, should be the same as specified in LOOPTIME (in s)
  //speed_pub.publish(&speed_msg);
  nh.spinOnce();
  nh.loginfo("Publishing odometry");
  }*/

void encoderLeftMotor()
{
  // Read the value for the encoder for the right wheel
  int val = digitalRead(left_encoderB);
  if (val == LOW)
  {
    if (left_wheel_tick_count == encoder_minimum)
    {
      left_wheel_tick_count = encoder_maximum;
    }
    else
    {
      left_wheel_tick_count--;
    }
  }
  else
  {
    if (left_wheel_tick_count == encoder_maximum)
    {
      left_wheel_tick_count = encoder_minimum;
    }
    else
    {
      left_wheel_tick_count++;
    }
  }
}
// Increment the number of pulses by 1
void encoderRightMotor()
{
  // Read the value for the encoder for the right wheel
  int val = digitalRead(right_encoderB);
  if (val == LOW)
  {
    if (right_wheel_tick_count == encoder_maximum)
    {
      right_wheel_tick_count = encoder_minimum;
    }
    else
    {
      right_wheel_tick_count++;
    }
  }
  else
  {
    if (right_wheel_tick_count == encoder_minimum)
    {
      right_wheel_tick_count = encoder_maximum;
    }
    else
    {
      right_wheel_tick_count--;
    }
  }
}

void playTone(int frequency, int duration) {
  tone(buzzer, frequency);
  delay(duration);
  noTone(buzzer);
}

template <typename T> int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}
