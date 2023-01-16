//include alle nodige h-files
#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/range.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif
#define echoPin 18 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin 5 //attach pin D3 Arduino to pin Trig of HC-SR04


#define echoPin1 21 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin1 22 //attach pin D3 Arduino to pin Trig of HC-SR04

#define echoPin2 9 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin2 10 //attach pin D3 Arduino to pin Trig of HC-SR04


int distance; // variable for the distance measurement
long duration; 
long duration1;
long duration2;

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg;

sensor_msgs__msg__Range range_msg;
sensor_msgs__msg__Range range_msg1;
sensor_msgs__msg__Range range_msg2;

rclc_executor_t executor;
rclc_executor_t executor_pub;
//rclc_executor_t executor_sub;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  while(1);
  
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  
  //vTaskDelay(100);
  //eerste message publishen
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &range_msg2, NULL));

    digitalWrite(trigPin2, LOW);
    delayMicroseconds(2);

    digitalWrite(trigPin2, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin2, LOW);

    duration2 = pulseIn(echoPin2, HIGH);

        range_msg2.range= duration2 * 0.034 / 2; 

    msg.data++;
  }

  //vTaskDelay(100);


  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &range_msg1, NULL));

    digitalWrite(trigPin1, LOW);
    delayMicroseconds(2);

    digitalWrite(trigPin1, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin1, LOW);

    duration1 = pulseIn(echoPin1, HIGH);

        range_msg1.range= duration1 * 0.034 / 2; 

    msg.data++;
  }

  //vTaskDelay(100);

  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &range_msg, NULL));

    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);

    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    duration = pulseIn(echoPin, HIGH);

        range_msg.range= duration * 0.034 / 2; 


    msg.data++;
  }
  
}



void setup() {
  // Configure serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT

  pinMode(trigPin1, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin1, INPUT); // Sets the echoPin as an INPUT

  pinMode(trigPin2, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin2, INPUT); // Sets the echoPin as an INPUT

  //delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
    "micro_ros_platformio_node_publisher"));

  

  


  

  // create timer,
  const unsigned int timer_timeout = 500;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
    range_msg.field_of_view = 0.26;
    range_msg.radiation_type= sensor_msgs__msg__Range__ULTRASOUND;
    range_msg.max_range = 400;
    range_msg.min_range = 2;
    range_msg1.max_range = 400;
    range_msg1.min_range = 2;
    range_msg2.max_range = 400;
    range_msg2.min_range = 2;


  msg.data = 0;
}

void loop() {
  
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}




