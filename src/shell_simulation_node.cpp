#include <shell_simulation/example_publisher.h>

double steering_value;

int main(int argc, char **argv) {
    ros::init(argc, argv, "shell_simulation");
    
    ExamplePublisher node;

    ros::Rate rate(10);

    int count = 0;
    const int max_count = 100;

    while (ros::ok() && count < max_count) {
        node.publish(steering_value);
        ros::spinOnce();
        rate.sleep();
        count++;
        steering_value = ((double)count - 50.0)/50;
    }

    return 0;
}
