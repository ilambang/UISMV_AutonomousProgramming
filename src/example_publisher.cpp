#include <shell_simulation/example_publisher.h>

#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

ExamplePublisher::ExamplePublisher() {
//    string_pub_ = nh_.advertise<std_msgs::String>("string", 1);
   string_pub_ = nh_.advertise<std_msgs::Float64>("steering_command", 1);
}

void ExamplePublisher::publish(double steering_val) {
    // std_msgs::String msg;
    std_msgs::Float64 msg;
    // msg.data = "Hello";
    // msg.data = 0.9;
    msg.data = steering_val;
    string_pub_.publish(msg);
}
