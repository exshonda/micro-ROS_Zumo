#include <Wire.h>
#include <ZumoShield.h>

#define BOARD_WITH_ESP_AT
#include <micro_ros_arduino.h>

#include <wifi_transport.cpp>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

//#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/int32.h>
#include <my_custom_message/msg/my_custom_message.h>
#include <geometry_msgs/msg/twist.h>

rcl_publisher_t publisher;
//sensor_msgs__msg__Imu pubmsg;
//std_msgs__msg__Int32 pubmsg;
my_custom_message__msg__MyCustomMessage pubmsg;
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist submsg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;

ZumoIMU imu;
ZumoReflectanceSensorArray reflectanceSensors;
ZumoMotors motors;

#define NUM_SENSORS 6
unsigned int sensorValues[NUM_SENSORS];

bool useEmitters = true;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){Serial.println(temp_rc); error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){Serial.println(temp_rc); error_loop();}}

void error_loop(){
  if (rcutils_error_is_set()){
    auto state = rcutils_get_error_state();

    Serial.print(state->file);
    Serial.print(" : ");
    Serial.print(state->line_number);
    Serial.print(" : ");
    Serial.println(state->message);
  } else {
    auto string = rcutils_get_error_string();
    Serial.println(string.str);
  }

  while(1){
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(500);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &pubmsg, NULL));

    imu.read();

    pubmsg.a.x = imu.a.x;
    pubmsg.a.y = imu.a.y;
    pubmsg.a.z = imu.a.z;
    pubmsg.g.x = imu.g.x;
    pubmsg.g.y = imu.g.y;
    pubmsg.g.z = imu.g.z;
    pubmsg.m.x = imu.m.x;
    pubmsg.m.y = imu.m.y;
    pubmsg.m.z = imu.m.z;

    reflectanceSensors.read(sensorValues, useEmitters ? QTR_EMITTERS_ON : QTR_EMITTERS_OFF);

    pubmsg.ls1 = sensorValues[0];
    pubmsg.ls2 = sensorValues[1];
    pubmsg.ls3 = sensorValues[2];
    pubmsg.ls4 = sensorValues[3];
    pubmsg.ls5 = sensorValues[4];
    pubmsg.ls6 = sensorValues[5];
  }
}

//twist message cb
void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  Serial.print("linear(");
  Serial.print(msg->linear.x);
  Serial.print(",");
  Serial.print(msg->linear.y);
  Serial.print(",");
  Serial.print(msg->linear.z);
  Serial.print("), angular(");
  Serial.print(msg->angular.x);
  Serial.print(",");
  Serial.print(msg->angular.y);
  Serial.print(",");
  Serial.print(msg->angular.z);
  Serial.println(")");
  motors.setSpeeds((int)(266 * msg->linear.x), (int)(266 * msg->linear.y));
  // if velocity in x direction is 0 turn off LED, if 1 turn on LED
  digitalWrite(LED_PIN, (msg->linear.z == 0) ? LOW : HIGH);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!imu.init())
  {
    Serial.println("Failed to initialize IMU sensors.");
    error_loop();
  }

  imu.enableDefault();
  reflectanceSensors.init();

  set_microros_wifi_transports("WIFI SSID", "WIFI PASS", "192.168.1.57", 8888);
  
  Serial.println("set_microros_wifi_transports");
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  Serial.println("rcl_get_default_allocator");
  
   //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  Serial.println("rclc_support_init");
  
  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  Serial.println("rclc_node_init_default");

  // create publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher,
    &node,
    //ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    //ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    ROSIDL_GET_MSG_TYPE_SUPPORT(my_custom_message, msg, MyCustomMessage),
    "topic_name"));
  
  Serial.println("rclc_publisher_init_best_effort");

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));
  
  Serial.println("rclc_timer_init_default");

  // create subscriber
  RCCHECK(rclc_subscription_init_best_effort(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  Serial.println("rclc_subscription_init_best_effort");

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));

  Serial.println("rclc_executor_init");

  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  Serial.println("rclc_executor_add_timer");

  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &submsg, &subscription_callback, ON_NEW_DATA));

  Serial.println("rclc_executor_add_subscription");
}

void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
