// Minimal Arduino Uno LED test over rosserial, using std_msgs/Bool.
//
//   led       (host -> board): std_msgs/Bool, true = LED on
//   led_state (board -> host): std_msgs/Bool, current LED level, 10 Hz
//
// Test with rosserial_server:
//   ros2 launch rosserial_server serial.launch.py port:=/dev/ttyACM0 baud:=57600
//   ros2 topic pub --once /led std_msgs/msg/Bool "{data: true}"
#include <Arduino.h>
#include <ros.h>
#include <std_msgs/msg/bool.hpp>

static const uint8_t LED_PIN = LED_BUILTIN;  // pin 13 on the Uno
static const unsigned long STATE_PERIOD_MS = 100;
static unsigned long next_period = 0;

ros::NodeHandle nh;

std_msgs::msg::Bool state_msg;
ros::Publisher state_pub("led_state", &state_msg);

void on_led(const std_msgs::msg::Bool & msg)
{
  digitalWrite(LED_PIN, msg.data ? HIGH : LOW);
}
ros::Subscriber<std_msgs::msg::Bool, void> led_sub("led", &on_led);

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  nh.initNode();
  nh.advertise(state_pub);
  nh.subscribe(led_sub);

  next_period = millis();
}

void loop()
{
  if (!nh.connected()) {
    next_period = millis();  // no burst of catch-up messages after (re)connecting
  } else if (millis() - next_period >= STATE_PERIOD_MS) {
    next_period += STATE_PERIOD_MS;
    state_msg.data = digitalRead(LED_PIN);
    state_pub.publish(&state_msg);
  }
  nh.spinOnce();
}
