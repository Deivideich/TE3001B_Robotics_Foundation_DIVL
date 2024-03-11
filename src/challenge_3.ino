#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>


rcl_subscription_t subscriber;
rcl_publisher_t publisher;
rcl_publisher_t publisher_voltage;
std_msgs__msg__Float32 voltage;
std_msgs__msg__Int32 msg;
std_msgs__msg__Float32 duty_cycle;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer_1;
rcl_timer_t timer_2;


#define LED_PIN 32
#define PWM_PIN 25

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer1_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    //read pot
    msg.data = analogRead(33);
  }
}

void timer2_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher_voltage, &voltage, NULL));
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    //publish voltage
    float raw_v = float(analogRead(33));
    voltage.data = map(raw_v, 0, 4096, 0, 3.3 * 100) / 100.0;
  }
}

void pwm_generator_callback(const void * duty_cycle)
{
  //suscribe to PWM generator
  const std_msgs__msg__Float32 * duty_cycle_ = (const std_msgs__msg__Float32 *)duty_cycle;
  int pwm_ = (duty_cycle_->data)*2.55;
  ledcWrite(0, pwm_);
}

void setup() {
  set_microros_transports();

  pinMode(PWM_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  digitalWrite(LED_PIN,HIGH);
  
  // Configuration of channel 0 with the chosen frequency and resolution
  ledcSetup(0, 5000, 8);
  ledcAttachPin(PWM_PIN, 0);



  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_esp32", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
            &publisher_voltage,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "/voltage"));

  RCCHECK(rclc_publisher_init_default(
            &publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "/raw_pot"));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
            &subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "pwm_duty_cycle"));

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
            &timer_1,
            &support,
            RCL_MS_TO_NS(10),
            timer1_callback));

  RCCHECK(rclc_timer_init_default(
            &timer_2,
            &support,
            RCL_MS_TO_NS(100),
            timer2_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_1));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_2));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &duty_cycle, &pwm_generator_callback, ON_NEW_DATA));

  duty_cycle.data = 0.0;
  voltage.data = 0;
  msg.data = 0;
}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
