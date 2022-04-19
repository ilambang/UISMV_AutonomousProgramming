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

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    double newX = msg->pose.pose.position.x;
    double newY = msg->pose.pose.position.y;

    ROS_INFO("newX = %f & newY = %f", newX, newY);
}
void speedoCallback(const std_msgs::Float32::ConstPtr& msg) {
    double speed = msg->data;

    ROS_INFO("speed = %f", speed);
}


int main(int argc, char **argv) {
    //Initialize the ros
    ros::init(argc, argv, "shell_simulation_node");

    //Creating Ros handle
    ros::NodeHandle n;

    //Publishe to certain topic
    brake_Pub = n.advertise<std_msgs::Float64>("brake_command", 8);
    throttle_Pub = n.advertise<std_msgs::Float64>("throttle_command", 8);
    steering_Pub = n.advertise<std_msgs::Float64>("steering_command", 8);
    gear_Pub = n.advertise<std_msgs::String>("gear_command", 8);
    handbrake_Pub = n.advertise<std_msgs::Bool>("handbrake_command", 8);
    
    //Subscribe to certain topic
    odometry_Sub = n.subscribe("carla/ego_vehicle/odometry", 8, odomCallback);
    speedometer_Sub = n.subscribe("carla/ego_vehicle/speedometer", 8, speedometerCallback);
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

    ros::spin();
    return 0;
}

// //misal ini adalah sebuah program

// #include <ros/ros.h>
// #include <stdlib.h>
// #include <string>
// #include <queue>
// #include <cmath>
// #include <std_msgs/Float64.h>
// #include <std_msgs/Float32.h>
// #include <std_msgs/String.h>
// #include <std_msgs/Bool.h>
// #include <nav_msgs/Odometry.h>

// #define PI 3.14159265358979323846
// #define PI_TIMES_2 6.283185
// #define PI_DIV_2 1.570796
// #define MAJU 0
// #define MUNDUR 1
// //#define LOCAL_COMMENTING true

// ros::Publisher throttlePublisher;
// ros::Publisher brakePublisher;
// ros::Publisher gearPublisher;
// ros::Publisher handbrakePublisher;
// ros::Publisher steeringPublisher;
// ros::Subscriber odomSubscriber;
// ros::Subscriber speedometerSubscriber;

// double lastThrottle = 0.0;
// double lastSteering = 0.0;
// bool lastHandbrake = 0.0;
// double lastBrake = 0.0;
// std::string lastGear = "forward";

// /* ======================================== */
// /* ===== functional publisher section ===== */
// /* ======================================== */
// void throttle(double x) {
//     // 0.0 to 1.0
//     if (lastThrottle == x) {
//         return;
//     }
//     lastThrottle = x;
//     std_msgs::Float64 msg;
//     msg.data = x;
//     throttlePublisher.publish(msg);
//     ros::spinOnce();
// }

// void brake(double x) {
//     // 0.0 to 1.0
//     if (lastBrake == x) {
//         return;
//     }
//     lastBrake = x;
//     std_msgs::Float64 msg;
//     msg.data = x;
//     brakePublisher.publish(msg);
//     ros::spinOnce();
// }

// void gear(std::string x) {
//     // "forward" or "reverse"
//     if (lastGear == x) {
//         return;
//     }
//     std_msgs::String msg;
//     msg.data = x;
//     lastGear = x;
//     gearPublisher.publish(msg);
//     ros::spinOnce();
// }

// void handbrake(bool x) {
//     // true or false
//     if (lastHandbrake == x) {
//         return;
//     }
//     lastHandbrake = x;
//     std_msgs::Bool msg;
//     msg.data = x;
//     handbrakePublisher.publish(msg);
//     ros::spinOnce();
// }

// void steering(double x) {
//     // -1.0 to 1.0
//     if (abs(lastSteering - x) < 0.0005) {
//         return;
//     }
//     lastSteering = x;
//     std_msgs::Float64 msg;
//     msg.data = x;
//     steeringPublisher.publish(msg);
//     ros::spinOnce();
// }
// /* ======================================== */


// /* ======================================== */
// /* ======= formula section ================ */
// /* ======================================== */
// double getDistance(double x1, double y1, double x2, double y2) {
//     return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
// }

// double getDistance_notsqrt(double x1, double y1, double x2, double y2) {
//     double p = x1 - x2, q = y1 - y2;
//     return p * p + q * q;
// }

// double getAngle(double x, double y) {
//     // mengembalikan sudut dari vektor (x, y) relatif terhadap sumbu x (0 <= sudut <= 2 * pi)
    
//     double res = std::acos((-x) / sqrt(x * x + y * y));
    
//     if (y < 0) {
//         res *= -1;
//     }
//     if (res < 0) {
//         res += 2 * PI;
//     }
//     return res;
// }
// /* ======================================== */


// /* ======================================== */
// /* === global vars and data type section == */
// /* ======================================== */
// struct Target {
//     double x;
//     double y;
//     int maju_mundur;
// };
// std::queue<Target> targetList;
// double lastX = -77.9;
// double lastY = -25;
// int cntTarget = 0;
// int needBrake = 0;
// double steerVal = 0.0;
// double steerMult = 1.0;
// double targetThrottle = 0.6;

// double vecX1;
// double vecY1;
// double vecX2;
// double vecY2;
// double angle1;
// double angle2;
// double angle;

// /* ======================================== */


// #ifdef LOCAL_COMMENTING
// /* ======================================== */
// /* ======= comment section ================ */
// /* ======================================== */
// #include <iostream>
// #include <chrono>
// #include <iomanip>

// struct Point {
//     double x;
//     double y;
// };
// Point checkpoints[12] = {
//     Point{0, 0},
//     Point{-135, 1.50}, 
//     Point{-212.00, -73.95},
//     Point{-212.00, -198.22}, 
//     Point{-135.50, -256.00}, 
//     Point{-84.5, -198.22}, 
//     Point{-135.50, -128.00}, 
//     Point{-135.50, -48.20}, 
//     Point{44.50, -65.75}, 
//     Point{0.0, -128.00}, 
//     Point{44.50, -198.22}, 
//     Point{0.0, -256.00}
// };
// bool isVisited[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
// int cntGoal = 0;
// void checkGoal(double x, double y) {
//     int i = 0;
//     for (Point checkpoint : checkpoints) {
//         double dist = getDistance(checkpoint.x, checkpoint.y, x, y);
//         if (dist <= 3.0) {
//             std::cout<<"!!! sekarang di ("<<x<<", "<<y<<") dengan jarak "<<dist<<" dari ckpt ("<<checkpoint.x<<", "<<checkpoint.y<<std::endl;
//             if (!isVisited[i]) {
//                 cntGoal++;
//                 isVisited[i] = 1;
//             }
//         }
//         i++;
//     }
// }

// double sumDistance = 0.0;
// bool isInitialized = false;
// double energy = 0.0;
// int localTargetCnt = -1;
// auto timeStart = std::chrono::system_clock::now();

// void commentSection() {
//     std::cout << std::fixed;
//     std::cout << std::setprecision(2);
//     auto now = std::chrono::system_clock::now();
//     double duration = (std::chrono::duration_cast<std::chrono::milliseconds>(now - timeStart)).count() / 1000.0;
//     if (targetList.empty()) {
//         std::cout<<"KELAR GAN, jarak: "<<sumDistance<<", waktu: "<<duration<<", energi: "<<(energy / 100000)<<", ckpt: "<<cntGoal<<std::endl;
//         return;
//     }
//     if (localTargetCnt != cntTarget) {
//         localTargetCnt = cntTarget;
//         std::cout<<"target: "<<cntTarget<<", jarak: "<<sumDistance<<", waktu: "<<duration<<", energi: "<<(energy / 100000)<<", ckpt: "<<cntGoal<<std::endl;
//     }
// }

// const double mass = 2176.46; // mass of the vehicle [kg]
// const double g = 9.81; // gravity constant [m/s^2]
// const double friction = 0.01; // rolling friction
// const double rho = 1.2; // for air at NTP [kg/m^3]
// const double drag_coeff = 0.357; // drag coefficient
// const double area = 3.389; // front area of vehicle [m^2] 
// double priorVelocity = 0.0;
// double lastSampledX = 0.0;
// double lastSampledY = 0.0;
// auto lastTime = std::chrono::system_clock::now();

// void calculateEnergy(const nav_msgs::Odometry::ConstPtr& msg, double newX, double newY) {
//     auto now = std::chrono::system_clock::now();
//     double dt = (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTime)).count() / 1000.0;
//     if (dt < 1.0) return;
//     lastTime = now;
    
//     double distance = getDistance(lastSampledX, lastSampledY, newX, newY);
//     double velocity = distance / dt;
//     double acceleration = (velocity - priorVelocity) / dt;
//     priorVelocity = velocity;
//     lastSampledX = newX;
//     lastSampledY = newY;
    
//     // calculate forces acting against the moving vehicle
//     double force = (mass * g * friction) + (0.5 * rho * drag_coeff * area * (velocity * velocity)) + (mass * acceleration);
    
//     // calculate energy used
//     double energyUsage = force * distance;
    
//     energy += energyUsage;
// }
// /* ======================================== */
// #endif


// /* ======================================== */
// /* ====== target definition section ======= */
// /* ======================================== */
// void initTarget() {
//     // menyimpan target
    
//     // mundur, belok kiri
//   	targetList.push(Target{-77.86, 16.8, MAJU});
//     targetList.push(Target{-77.86, 30, MAJU});
//     targetList.push(Target{-77.86, 194.16, MAJU});
//     targetList.push(Target{-15.45, 194.16, MAJU});
// }
// /* ======================================== */


// /* ======================================== */
// /* ======= callback section =============== */
// /* ======================================== */
// void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
//     double newX = msg->pose.pose.position.x;
//     double newY = msg->pose.pose.position.y;
    
//     #ifdef LOCAL_COMMENTING
//         checkGoal(newX, newY);
//         if (isInitialized) {
//             sumDistance += getDistance(lastX, lastY, newX, newY);
//         } else {
//             isInitialized = true;
//         }
//         calculateEnergy(msg, newX, newY);
//         commentSection();
//     #endif
    
//     if (newX == lastX && newY == lastY) {
//         // kalau belum berubah posisi, tidak perlu melakukan apa-apa
//         return;
//     }
//     if (targetList.empty()) {
//         // update value global vars
//         lastX = newX;
//         lastY = newY;
//         return;
//     }
//     // sumDistance += getDistance(newX, newY, lastX, lastY);
//     // jika tidak ada target, maka berhenti
//     // jika ada target, maka cek apakah kita sedang menuju ke target atau malah sudah menjauhi target
//     // jika kita sudah menjauhi target, maka kita perlu menuju target selanjutnya
//     // jika masih ada target, maka kita harus berusaha mendekati target
//     Target target = targetList.front();
//     double distance = getDistance_notsqrt(target.x, target.y, newX, newY);
//     if (distance < 25) {
//         targetList.pop();
//         cntTarget++;
//     }
//     if (targetList.empty()){
//         throttle(0.0);
//         brake(1.0);
//         handbrake(true);
//         #ifdef LOCAL_COMMENTING
//             commentSection();
//         #endif
//         ros::Duration(1.0).sleep();
//         return;
//     }
    
//     target = targetList.front();
    
//     switch(target.maju_mundur) {
//         case MAJU:
//             gear("forward");
//             steerMult = 1.0;
//             break;
//         case MUNDUR:
//             gear("reverse");
//             steerMult = -1.0;
//             break;
//     }
//     vecX1 = newX - lastX;
//     vecY1 = newY - lastY;
//     vecX2 = target.x - newX;
//     vecY2 = target.y - newY;
//     angle1 = getAngle(vecX1, vecY1);
//     angle2 = getAngle(vecX2, vecY2);
//     angle = angle2 - angle1;
    
//     // konversi 0 <= angle <= 2*PI ke -PI < angle <= PI
//     // berguna untuk mencari steeringValue
//     if (angle > PI) { 
//         angle -= PI_TIMES_2;
//     }
//     if (angle <= -1 * PI) {
//         angle += PI_TIMES_2;
//     }
            
//     // value dari steering, dihitung untuk rentang -1.0 (full right) sampai 1.0 (full left)
//     // asumsi itu untuk sudut 90 derajat hingga -90 derajat
//     steerVal = angle / (PI_DIV_2);
//     if (steerVal < -0.65) {
//         steerVal = -1.0;
//     }
//     if (steerVal > 0.65) {
//         steerVal = 1.0;
//     }
//     steering(steerVal * steerMult);    
//     // update value global vars
//     lastX = newX;
//     lastY = newY;
// }

// void speedometerCallback (const std_msgs::Float32::ConstPtr& msg) {
//     if (msg.data >= 10) {
//         targetThrottle = 0.3;
//     } else {
//         targetThrottle = 0.6;
//     }
//     throttle(targetThrottle);
// }
// /* ======================================== */


// /* ======================================== */
// /* ========== main section ================ */
// /* ======================================== */
// int main(int argc, char **argv) {
//     ros::init(argc, argv, "shell_simulation_node");
//     ros::NodeHandle nh;
//     throttlePublisher = nh.advertise<std_msgs::Float64>("throttle_command", 8);
//     brakePublisher = nh.advertise<std_msgs::Float64>("brake_command", 8);
//     gearPublisher = nh.advertise<std_msgs::String>("gear_command", 8);
//     handbrakePublisher = nh.advertise<std_msgs::Bool>("handbrake_command", 8);
//     steeringPublisher = nh.advertise<std_msgs::Float64>("steering_command", 8);
//     initTarget();
//     ros::Duration(1.0).sleep(); //sleep for a second to ensure the publisher is connected to subscriber
//     odomSubscriber = nh.subscribe("carla/ego_vehicle/odometry", 8, odomCallback);
//     speedometerSubscriber = nh.subscribe("carla/ego_vehicle/speedometer", 8, speedometerCallback);
//     ros::spin();
// }
// /* ======================================== */

// git config --global http.proxy http://152.118.148.7:3128