#include <ros/ros.h>

// double steering_val;

class ExamplePublisher {
  public:
    ExamplePublisher();

    void publish(double steering_val);
    

  private:
    ros::NodeHandle nh_;

    ros::Publisher string_pub_;
};
