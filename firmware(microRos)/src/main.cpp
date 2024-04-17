// include needed library (arduino & mircorros)
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// include needed msg types
#include <geometry_msgs/msg/twist.h>
#include "tutorial_interfaces/msg/control.h"
#include "tutorial_interfaces/msg/carspeed.h"

//include self-created libraries and settings
#include "settings.h"
#include "Motor.h"
#include <ESP32Servo.h>

// create objects for micro-ros
rcl_publisher_t publisher;
rcl_subscription_t subscriber_AI;
rcl_subscription_t subscriber_controller;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator = rcl_get_default_allocator();
rcl_node_t node;
rcl_timer_t timer;

// create msgs 
tutorial_interfaces__msg__Carspeed pub_msg;
tutorial_interfaces__msg__Control sub_AI_msg;
geometry_msgs__msg__Twist sub_control_msg;

// Motor object and Servo object
Motor Left_Motor = Motor (L_R_PWM_pin, L_L_PWM_pin, L_A_encoder_pin, L_B_encoder_pin);
Motor Right_Motor = Motor(R_R_PWM_pin, R_L_PWM_pin, R_A_encoder_pin, R_B_encoder_pin);
Servo tilt;
Servo pan;

// for PID calculation
float order_x = 0;
float order_z = 0;
float order_speed_left = 0;
float order_speed_right = 0;

// for turret control
bool ai_order = 0;
int order_pan = 0;
int order_tilt = 0;
int tilt_angle = 0;
int pan_angle = 0;

// debug variable
int degree = 0;
int up_tilt = 1;
int up_pan = 1;

// for fire 
int fire = 0;

// Interrupt variables
volatile long pos_b_l = 0;
volatile long pos_b_r = 0;
float pos_b_l_prev = 0;
float pos_b_r_prev = 0;


// define patterns for simplier code
#define RCCHECK(fn)                                                                                                    \
  {                                                                                                                    \
    rcl_ret_t temp_rc = fn;                                                                                            \
    if ((temp_rc != RCL_RET_OK))                                                                                       \
    {                                                                                                                  \
      error_loop();                                                                                                    \
    }                                                                                                                  \
  }
#define RCSOFTCHECK(fn)                                                                                                \
  {                                                                                                                    \
    rcl_ret_t temp_rc = fn;                                                                                            \
    if ((temp_rc != RCL_RET_OK))                                                                                       \
    {                                                                                                                  \
    }                                                                                                                  \
  }

///////////////////////////////////////////////////////////////////////////////////////////////
// Interrupt functions
// Left encoder
void read_L_encoder_a()
{
  if (digitalRead(L_A_encoder_pin) == HIGH){
    if (digitalRead(L_B_encoder_pin) == HIGH)
      pos_b_l++;
    else
      pos_b_l--;
  }
  else{
    if (digitalRead(L_B_encoder_pin) == LOW)
      pos_b_l++;
    else
      pos_b_l--;
  }
}

void read_L_encoder_b()
{
  if (digitalRead(L_B_encoder_pin) == HIGH)
  {
    if (digitalRead(L_A_encoder_pin) == HIGH)
      pos_b_l++;
    else
      pos_b_l--;
  }
  else
  {
    if (digitalRead(L_A_encoder_pin) == LOW)
      pos_b_l++;
    else
      pos_b_l--;
  }
}

// Right encoder
void read_R_encoder_a()
{
  if (digitalRead(R_A_encoder_pin) == HIGH){
    if (digitalRead(R_B_encoder_pin) == HIGH)
      pos_b_r--;
    else
      pos_b_r++;
  }
  else{
    if (digitalRead(R_B_encoder_pin) == HIGH)
      pos_b_r--;
    else
      pos_b_r++;
  }
}

void read_R_encoder_b()
{
  if (digitalRead(R_B_encoder_pin) == HIGH)
  {
    if (digitalRead(R_A_encoder_pin) == HIGH)
      pos_b_r--;
    else
      pos_b_r++;
  }
  else
  {
    if (digitalRead(R_A_encoder_pin) == HIGH)
      pos_b_r--;
    else

      pos_b_r++;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////

// Error handle loop
void error_loop()
{
  while (1)
    ;
}

// configure the Carspeed msg to be publish
void create_pub_msg()
{
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  pub_msg.left_wheel_speed = pan.read();
  pub_msg.right_wheel_speed = tilt.read();
  return;
}

// timer callback function
void timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
  Serial.println("enter timer callback");
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
    RCSOFTCHECK(rcl_publish(&publisher, &pub_msg, NULL));
    //debug

}

// subscriber callback function
void subs_AI_callback(const void* msgin){
  tutorial_interfaces__msg__Control* msg = (tutorial_interfaces__msg__Control*) msgin;
  
  //unpack turret order, only when ai is not sending 999 (-1 means ai does not detect target)
  if (msg -> pan_angle != -999){
    order_pan = msg->pan_angle;
    order_tilt = msg->tilt_angle;
  }
  else {
    Serial.println("no orders");

    if (pan_angle == 180)
      up_pan = 0;
    else if (pan_angle == 0)
      up_pan = 1;
    if (up_pan == 1)
      order_pan = 1;
    else 
      order_pan = -1;

    if (tilt_angle == 180)
      up_tilt = 0;
    else if (tilt_angle == 0)
      up_tilt = 1;
    if (up_tilt == 1)
      order_tilt = 1;
    else 
      order_tilt = -1;
  }

  // unpack fire
  fire = msg->fire;

  // laser
  if (fire == 1)
    digitalWrite(Laser_pin, LOW);
  else
    digitalWrite(Laser_pin, HIGH);

}

void subs_controller_callback(const void* msgin){
  geometry_msgs__msg__Twist* msg = (geometry_msgs__msg__Twist*) msgin;
  Serial.println("controller_msgin recieved");

  order_x = msg->linear.x;
  order_z = msg->angular.z;
}

void setMotor(){
  order_speed_left = order_x - (order_z * 0.23/0.7);
  order_speed_right = order_x + (order_z * 0.23/0.7);

  if (order_speed_left < 0 )
    Left_Motor.PID.SetControllerDirection(Left_Motor.PID.Action::reverse);
  else
    Left_Motor.PID.SetControllerDirection(Left_Motor.PID.Action::direct);
  if (order_speed_right < 0)
    Right_Motor.PID.SetControllerDirection(Right_Motor.PID.Action::reverse);
  else
    Right_Motor.PID.SetControllerDirection(Right_Motor.PID.Action::direct);

  Left_Motor.target = order_speed_left * 10;
  Right_Motor.target = order_speed_right* 10;
  Left_Motor.input = pos_b_l - pos_b_l_prev;
  Right_Motor.input = pos_b_r - pos_b_r_prev;

  Left_Motor.PID.Compute();
  Right_Motor.PID.Compute();

  // make down current value for next calculation
  pos_b_l_prev = pos_b_l;
  pos_b_r_prev = pos_b_r;

  // move the motors
  Left_Motor.setMotor(Left_Motor.output);
  Right_Motor.setMotor(Right_Motor.output);
  Serial.print("left PWM: ");
  Serial.println(Left_Motor.output);
  Serial.print("right PWM: ");
  Serial.println(Right_Motor.output);
}

void setTurret(){
  Serial.print("order_pan: ");
  Serial.println(order_pan);
  Serial.print("order_tilt: ");
  Serial.println(order_tilt);

  tilt.write(constrain(order_tilt + tilt_angle, 0, 180));
  pan.write(constrain(order_pan + pan_angle, 0, 180));

  tilt_angle = constrain(order_tilt + tilt_angle, 0, 180);
  pan_angle = constrain(order_pan + pan_angle, 0, 180);

}

//********************************************************************************************************************************************
void setup()
{
  Serial.begin(115200);

  // FreeNetwork5g
  IPAddress agent_ip(192, 168, 63, 93);

  size_t agent_port = 8888;

  char ssid[] = "FreeNetwork5g";
  char psk[]= "wqrs2613";

  Serial.println("Attempting to connect to WiFi");
  Serial.print("ssid: ");
  Serial.println(ssid);
  // set micro ros to use WiFi as transport
  set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
  Serial.println("WiFi set up");

  // Configure pins
  attachInterrupt(digitalPinToInterrupt(L_A_encoder_pin), read_L_encoder_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(L_B_encoder_pin), read_L_encoder_b, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_A_encoder_pin), read_R_encoder_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_B_encoder_pin), read_R_encoder_b, CHANGE);
  
  // set turnings (pid contants) and QuickPID mode
  // ignore error msg for Control::automatic, this is from offical example of QuickPID
  // and it compliles and runs
  Left_Motor.PID.SetTunings(left_kp, left_ki, left_kd);
  Left_Motor.PID.SetMode(Left_Motor.PID.Control::automatic);
  Right_Motor.PID.SetTunings(right_kp, right_ki, right_kd);
  Right_Motor.PID.SetMode(Right_Motor.PID.Control::automatic);

  // set Servo configuration
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  tilt.setPeriodHertz(300);
  pan.setPeriodHertz(300);

  tilt.attach(Servo_tilt_pin, minUs, maxUs);
  pan.attach(Servo_pan_pin, minUs, maxUs);
  tilt.write(0);
  pan.write(0);

  // Set laser pin to high as signal pin now work as GND
  pinMode(Laser_pin, OUTPUT);
  digitalWrite(Laser_pin, HIGH);
  Serial.println("pin set up complete");
  
  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  Serial.println("init options set up complete");

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_node", "", &support));
  Serial.println("node set up complete");

  // create publisher for pc side
  RCCHECK(rclc_publisher_init_default(
      &publisher, 
      &node, 
      ROSIDL_GET_MSG_TYPE_SUPPORT(tutorial_interfaces, msg, Carspeed),
      "micro_ros_node_pub"));
  Serial.println("publisher set up complete");


  // create subscribers to pc side
  RCCHECK(rclc_subscription_init_default(
    &subscriber_AI,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(tutorial_interfaces, msg, Control),
    "/pc_side_pub"));
  Serial.println("subs AI set up complete");

  // create subsccriber to controller 
  RCCHECK(rclc_subscription_init_default(
    &subscriber_controller,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel"));
  Serial.println("subs controller set up complete");


  // create executor
  RCCHECK(rclc_executor_init(
    &executor, 
    &support.context, 
    2, 
    &allocator));
  Serial.println("executor set up complete");

  // add subsccriber and timer to the excutor
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_AI, &sub_AI_msg, &subs_AI_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_controller, &sub_control_msg, &subs_controller_callback, ON_NEW_DATA));
  Serial.println("excutor add AI complete");
  Serial.println("excutor add timer complete");

  Serial.println("executor add set up complete");
  delay_local(1000);
}

void loop()
{
  Serial.println("hello");
  // node run for 10ms
  RCSOFTCHECK(rclc_executor_spin_some(&executor, 1000 * 1000 * 10));
  setMotor();
  setTurret();
}