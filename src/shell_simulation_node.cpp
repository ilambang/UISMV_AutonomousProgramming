/*
    Shell Simulation
    by Iman Herlambang Suherman, Adiro Pradaya Gahana, Ahmad Akbar Habibillah
    Nakoela Team

*/

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

/*Define Constants*/
#define PI 3.14159265358979323846
#define PI_MULT_2 6.283185
#define PI_DIV_2 1.570796
#define FORWARD 0
#define BACKWARD 1

/*Create object for publishing data or publisher*/
ros::Publisher brake_Pub;
ros::Publisher throttle_Pub;
ros::Publisher steering_Pub;
ros::Publisher gear_Pub;
ros::Publisher handbrake_Pub;

/*Create object for subscribing sensor data*/
ros::Subscriber odometry_Sub;
ros::Subscriber speedometer_Sub;

/* ======================================== */
/* === global vars and data type section == */
/* ======================================== */
struct coordQueue {
    double x;
    double y;
    int fwrd_bwrd;
};
std::queue<coordQueue> targetList;
double lastX = -77.9;
double lastY = -25;
int cntTarget = 0;
int needBrake = 0;
double steerVal = 0.0;
double steerMult = 1.0;
double targetThrottle = 0.6;

double last_throttle_value = 0.0;
double last_steering_value = 0.0;
bool last_handbrake_value = 0.0;
double last_brake_value = 0.0;
std::string last_gear_value = "forward";

double angle;

/* Publisher Functions  */


void set_throttle(double x) {
    // range: 0.0 to 1.0
    if (last_throttle_value == x) {
        return;
    }

    last_throttle_value = x;
    std_msgs::Float64 msg;
    msg.data = x;
    throttle_Pub.publish(msg);
    ros::spinOnce();
}

void set_brake(double x) {
    // range: 0.0 to 1.0
    if (last_brake_value == x) {
        return;
    }
    last_brake_value = x;
    std_msgs::Float64 msg;
    msg.data = x;
    brake_Pub.publish(msg);
    ros::spinOnce();
}

void set_gear(std::string x) {
    // "forward" or "reverse"
    if (last_gear_value == x) {
        return;
    }
    std_msgs::String msg;
    msg.data = x;
    last_gear_value = x;
    gear_Pub.publish(msg);
    ros::spinOnce();
}

void set_handbrake(bool x) {
    // true or false
    if (last_handbrake_value == x) {
        return;
    }
    last_handbrake_value = x;
    std_msgs::Bool msg;
    msg.data = x;
    handbrake_Pub.publish(msg);
    ros::spinOnce();
}

void set_steering(double x) {
    // -1.0 to 1.0
    if (abs(last_steering_value - x) < 0.0005) {
        return;
    }
    last_steering_value = x;
    std_msgs::Float64 msg;
    msg.data = x;
    steering_Pub.publish(msg);
    ros::spinOnce();
}

/* ======================================== */
/* ======= formula section ================ */
/* ======================================== */

double CalcDistance(double x1, double y1, double x2, double y2) {
    // returns distance between two vectors
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

double CalcDistance_notsqrt(double x1, double y1, double x2, double y2) {
    // returns distance between two vectors (without sqrt function)
    double p = x1 - x2, q = y1 - y2;
    return p * p + q * q;
}

double CalcAngle(double x, double y) {
    // returns angle from vector (x,y) relative to Totalbu x (0 <= angle <= 2*pi)
    double res = std::acos((-x) / sqrt(x * x + y * y));
    
    if (y < 0) {
        res *= -1;
    }
    if (res < 0) {
        res += 2 * PI;
    }
    return res;
}
/* ======================================== */

#ifdef LOCAL_COMMENTING
/* ======================================== */
/* ======= comment section ================ */
/* ======================================== */
#include <iostream>
#include <chrono>
#include <iomanip>

struct Point {
    double x;
    double y;
};
Point checkpoints[12] = {
    Point{0, 0},
    Point{-135, 1.50}, 
    Point{-212.00, -73.95},
    Point{-212.00, -198.22}, 
    Point{-135.50, -256.00}, 
    Point{-84.5, -198.22}, 
    Point{-135.50, -128.00}, 
    Point{-135.50, -48.20}, 
    Point{44.50, -65.75}, 
    Point{0.0, -128.00}, 
    Point{44.50, -198.22}, 
    Point{0.0, -256.00}
};
bool isVisited[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int Goal = 0;
void checkGoal(double x, double y) {
    int i = 0;
    for (Point checkpoint : checkpoints) {
        double dist = CalcDistance(checkpoint.x, checkpoint.y, x, y);
        if (dist <= 3.0) {
            std::cout<<"!!! now at ("<<x<<", "<<y<<") with distance "<<dist<<" from ckpt ("<<checkpoint.x<<", "<<checkpoint.y<<std::endl;
            if (!isVisited[i]) {
                Goal++;
                isVisited[i] = 1;
            }
        }
        i++;
    }
}

double TotalDistance = 0.0;
bool isInitialized = false;
double Energy = 0.0;
int localTargetCnt = -1;
auto Start = std::chrono::system_clock::now();

void commentSection() {
    std::cout << std::fixed;
    std::cout << std::setprecision(2);
    auto now = std::chrono::system_clock::now();
    double duration = (std::chrono::duration_cast<std::chrono::milliseconds>(now - Start)).count() / 1000.0;
    if (coordQueue.empty()) {
        std::cout<<"Finish, jarak: "<<TotalDistance<<", time: "<<duration<<", energy: "<<(Energy / 100000)<<", ckpt: "<<Goal<<std::endl;
        return;
    }
    if (localTargetCnt != cntTarget) {
        localTargetCnt = cntTarget;
        std::cout<<"target: "<<cntTarget<<", distance: "<<TotalDistance<<", waktu: "<<duration<<", energi: "<<(Energy / 100000)<<", ckpt: "<<Goal<<std::endl;
    }
}

const double mass = 2176.46; // mass of the vehicle [kg]
const double g = 9.81; // gravity constant [m/s^2]
const double friction = 0.01; // rolling friction
const double rho = 1.2; // for air at NTP [kg/m^3]
const double drag_coeff = 0.357; // drag coefficient
const double area = 3.389; // front area of vehicle [m^2] 
double priorVelocity = 0.0;
double lastSampledX = 0.0;
double lastSampledY = 0.0;
auto lastTime = std::chrono::system_clock::now();

void calculateEnergy(const nav_msgs::Odometry::ConstPtr& msg, double newX, double newY) {
    auto now = std::chrono::system_clock::now();
    double dt = (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTime)).count() / 1000.0;
    if (dt < 1.0) return;
    lastTime = now;
    
    double distance = CalcDistance(lastSampledX, lastSampledY, newX, newY);
    double velocity = distance / dt;
    double acceleration = (velocity - priorVelocity) / dt;
    priorVelocity = velocity;
    lastSampledX = newX;
    lastSampledY = newY;
    
    // calculate forces acting against the moving vehicle
    double force = (mass * g * friction) + (0.5 * rho * drag_coeff * area * (velocity * velocity)) + (mass * acceleration);
    
    // calculate Energy used
    double EnergyUsage = force * distance;
    
    Energy += EnergyUsage;
}
/* ======================================== */
#endif

void initTarget() {
    // menyimpan target
    
    // belok kiri segitiga
    targetList.push(coordQueue{-93, -13, FORWARD});
    targetList.push(coordQueue{-100, -18, FORWARD});
    targetList.push(coordQueue{-129, -45.5, FORWARD});
    targetList.push(coordQueue{-143, -55, FORWARD});
    targetList.push(coordQueue{-145.75, -75.7, FORWARD});

    // BACKWARD, belok kiri
    targetList.push(coordQueue{-145.47, -7.79, BACKWARD});
    targetList.push(coordQueue{-145.47, -0.9, BACKWARD});
    targetList.push(coordQueue{-145.47, -0.9, BACKWARD});
    targetList.push(coordQueue{-104.58, -0.5, BACKWARD});
    targetList.push(coordQueue{-52.68, -0.91, BACKWARD});

    // FORWARD lawan arah, belok kanan, lurus
    targetList.push(coordQueue{-77.86, -0.91, FORWARD});
    targetList.push(coordQueue{-77.86, 20, FORWARD});
    
    // FORWARD, belok kanan
    targetList.push(coordQueue{-77.86, 145, FORWARD});
    targetList.push(coordQueue{-77, 171, FORWARD});
    targetList.push(coordQueue{-75.1, 181.2, FORWARD});
    targetList.push(coordQueue{-64, 193, FORWARD});
    targetList.push(coordQueue{-53.3, 194.16, FORWARD});
    targetList.push(coordQueue{-15.45, 194.16, FORWARD});
    
    //belok kanan lurus
    targetList.push(coordQueue{-6.1, 191.5, FORWARD});
    targetList.push(coordQueue{-2.6, 185.2, FORWARD});
    targetList.push(coordQueue{-3.3, 177.4, FORWARD});
    targetList.push(coordQueue{-3.7, 151.0, FORWARD});
    targetList.push(coordQueue{-4.32, 110.51, FORWARD});
    targetList.push(coordQueue{-6.7, 34.8, FORWARD});
    
    //bunderan
    targetList.push(coordQueue{-9.8, 23.9, FORWARD});
    targetList.push(coordQueue{-18.4, 8.9, FORWARD});
    targetList.push(coordQueue{-21.7, -1.2, FORWARD});
    targetList.push(coordQueue{-19.2, -10.4, FORWARD});
    targetList.push(coordQueue{-12.9, -17.6, FORWARD});
    targetList.push(coordQueue{-2.3, -22.1, FORWARD});
    targetList.push(coordQueue{9.3, -19.4, FORWARD});
    targetList.push(coordQueue{19.3, -10.0, FORWARD});
    targetList.push(coordQueue{24.6, -7.3, FORWARD});
    
    //lurus, belok kanan
    targetList.push(coordQueue{30.5, -7.1, FORWARD});
    targetList.push(coordQueue{220.1, -9.7, FORWARD});
    targetList.push(coordQueue{226.4, -13.1, FORWARD});
    targetList.push(coordQueue{230.8, -19.1, FORWARD});
    targetList.push(coordQueue{231.1, -24.9, FORWARD});
    
    //belok kanan, lurus
    targetList.push(coordQueue{230.7, -45.8, FORWARD});
    targetList.push(coordQueue{226.9, -52.8, FORWARD});
    targetList.push(coordQueue{220.7, -57.3, FORWARD});
    targetList.push(coordQueue{213.8, -58.1, FORWARD});
    
    //belok kiri
    targetList.push(coordQueue{180.9, -58.3, FORWARD});
    targetList.push(coordQueue{170.1, -65.0, FORWARD});
    targetList.push(coordQueue{167.2, -73.6, FORWARD});
    targetList.push(coordQueue{167.1, -87.5, FORWARD});
    
    //belok kanan panjang
    targetList.push(coordQueue{166.5, -96.9, FORWARD});
    targetList.push(coordQueue{164.1, -105.4, FORWARD});
    targetList.push(coordQueue{158.4, -115.1, FORWARD});
    targetList.push(coordQueue{154.0, -120.2, FORWARD});
    targetList.push(coordQueue{146.8, -125.1, FORWARD});
    targetList.push(coordQueue{139.7, -127.8, FORWARD});
    targetList.push(coordQueue{133.2, -128.8, FORWARD});
    
    //lurus, belok kiri
    targetList.push(coordQueue{5.2, -130.7, FORWARD});
    targetList.push(coordQueue{-5.5, -135.1, FORWARD});
    targetList.push(coordQueue{-8.6, -141.6, FORWARD});
    
    //lurus, belok kanan, lurus
    targetList.push(coordQueue{-8.9, -147.5, FORWARD});
    targetList.push(coordQueue{-8.8, -188.6, FORWARD});
    targetList.push(coordQueue{-9.2, -193.1, FORWARD});
    targetList.push(coordQueue{-13.3, -196.0, FORWARD});
    targetList.push(coordQueue{-19.2, -196.9, FORWARD});
    targetList.push(coordQueue{-38.7, -196.9, FORWARD});
    targetList.push(coordQueue{-50.6, -193.6, FORWARD});
}


void movetoNewCoords(const nav_msgs::Odometry::ConstPtr& msg) {
    double coordsX = msg->pose.pose.position.x;
    double coordsY = msg->pose.pose.position.y;

    if (coordsX == lastX && coordsY == lastY) {
        // kalau belum berubah posisi, tidak perlu melakukan apa-apa
        return;
    }
    if (targetList.empty()) {
        // update value global vars
        lastX = coordsX;
        lastY = coordsY;
        return;
    }

    ROS_INFO("newX = %f & newY = %f", coordsX, coorsY);

    coordQueue target = targetList.front(); //Find most upper queue
    double distance = CalcDistance_notsqrt(target.x, target.y, newX, newY);

    if(distance < 20) { //safe distance for the confirmed checkpoint
        coordQueue.pop();
    }

    if (coordQueue.empty()){
        set_throttle(0.0)
        set_brake(1.0);
        set_handbrake(true);
    }

    switch(target.fwrd_bwrd) {
        case FORWARD:
            set_gear("forward");
            steerMult = 1.0;
            break;
        case BACKWARD:
            set_gear("reverse");
            steerMult = -1.0;
            break;
            
    angle = CalcAngle(target.x - coordsX, target.y - coordsY); - CalcAngle(coordsX - lastX, coordsY - lastY);
    
    // konversi 0 <= angle <= 2*PI ke -PI < angle <= PI
    // berguna untuk mencari steeringValue
    if (angle > PI) { 
        angle -= PI_TIMES_2;
    }
    else if (angle <= -1 * PI) {
        angle += PI_TIMES_2;
    }
            
    steerVal = angle / (PI_DIV_2);
    if (steerVal < -0.65) {
        steerVal = -1.0;
    }
    else if (steerVal > 0.65) {
        steerVal = 1.0;
    }
    steering(steerVal * steerMult);  
      
    // update value global vars
    lastX = coordsX;
    lastY = coordsY;
    }


    
}
void speedometerCallback(const std_msgs::Float32::ConstPtr& msg) {
    if (msg->data >= 10) {
        targetThrottle = 0.3;
    } else {
        targetThrottle = 0.6;
    }
    throttle(targetThrottle);

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
    
    initTarget();
    //Subscribe to certain topic
    ros::Duration(1.0).sleep(); 
    odometry_Sub = n.subscribe("carla/ego_vehicle/odometry", 8, movetoNewCoords);
    speedometer_Sub = n.subscribe("carla/ego_vehicle/speedometer", 8, speedometerCallback);
    ros::Rate rate(10);
    ros::spin();
    return 0;
}

// //misal ini adalah sebuah program



// git config --global http.proxy http://152.118.148.7:3128