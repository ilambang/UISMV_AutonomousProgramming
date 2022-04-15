/*Include ROS library and it's dependency*/
#include <ros/ros.h>
#include <stdlib.h>
#include <string>
#include <queue>
#include <cmath>

/*Include standard format library*/
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>

/*Create object for publishing data or publisher*/
ros::Publisher brake_Pub;
ros::Publisher throttle_Pub;
ros::Publisher steering_Pub;
ros::Publisher gear_Pub;
ros::Publisher handbrake_Pub;

/*Create object for subscribing sensor data*/
ros::Subscriber odometry_Sub;
ros::Subscriber speedometer_Sub;


double steering_value;
float lat,longitude,alt;

// void chatterCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
// {
//   ROS_INFO("I heard: [%s]", msg->data.c_str());
//   std::cout << msg->longitude << msg->latitude << msg->altitude << std::endl;
// }

int main(int argc, char **argv) {
    //Initialize the ros
    ros::init(argc, argv, "shell_simulation_node");

    //Creating Ros handle
    ros::NodeHandle n;

    //Subsribe to certain topic
    brake_Pub = n.advertise<std_msgs::Float64>("brake_command", 8);
    throttle_Pub = n.advertise<std_msgs::Float64>("throttle_command", 8);
    steering_Pub = n.advertise<std_msgs::Float64>("steering_command", 8);
    gear_Pub = n.advertise<std_msgs::String>("gear_command", 8);
    handbrake_Pub = n.advertise<std_msgs::Bool>("handbrake_command", 8);
    

    ros::Rate rate(10);

    int count = 0;
    const int max_count = 100;

    std_msgs::Float64 msg;

    while (ros::ok() && count < max_count) {
        
        msg.data = steering_value;
        steering_Pub.publish(msg);
        ros::spinOnce();
        ros::spinOnce();
        rate.sleep();
        count++;
        steering_value = ((double)count - 50.0)/50;
    }

    return 0;
}

//misal ini adalah sebuah program