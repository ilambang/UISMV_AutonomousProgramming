#include <ros/ros.h>
#include <string>
#include <cmath>
#include <vector>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <carla_msgs/CarlaLaneInvasionEvent.h>

ros::Publisher throttlePublisher;
ros::Publisher brakePublisher;
ros::Publisher gearPublisher;
ros::Publisher steeringPublisher;
ros::Subscriber odomSubscriber;
ros::Subscriber speedometerSubscriber;
ros::Subscriber lane_invasionSubscriber;

double Throttle_0 = 0.0;
double Steering_0 = 0.0;
double Brake_0=0.0;
std::string lastGear = "forward";


void throttle(double x) {
    Throttle_0 = x;
    std_msgs::Float64 msg;
    msg.data = x;
    throttlePublisher.publish(msg);
    ros::spinOnce();
}


void gear(std::string x) {
    std_msgs::String msg;
    msg.data = x;
    lastGear = x;
    gearPublisher.publish(msg);
    ros::spinOnce();
}

void steering(double x) {
    if (abs(Steering_0 - x) < 0.01){
		return;
	}
    Steering_0 = x;
    std_msgs::Float64 msg;
    msg.data = x;
    steeringPublisher.publish(msg);
    ros::spinOnce();
}
void brake(double x) {
    Brake_0 = x;
    std_msgs::Float64 msg;
    msg.data = x;
    brakePublisher.publish(msg);
    ros::spinOnce();
}


std::vector<double> X_obj;
std::vector<double> Y_obj;
double x_0 = -77.9;
double y_0 = -17.59;
int count = 0;
double Steering_X = 0.0;
double Throttle_X;


void maju() {
    gear("forward");
    steering(Steering_X);
    Throttle_X=0.52;
    Brake_0=0.0;
}

void mundur() {
    gear("reverse");
    steering(-1*Steering_X);
    Throttle_X=0.45;
    Brake_0=0.0;
}
void rem(){
	Steering_X=0.0;
	Throttle_X=0.0;	
	brake(1.0);
}

void setCommand() {
    switch(count) {
        case 1:
        case 7:
        rem();
        break;
        
        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
        mundur();
        break;
        
        default:
            maju();
            break;
    }
}

void initTarget() {

    //lurus ckpt1
    X_obj.push_back(-77.86); Y_obj.push_back(16.80);//0 77.86
  
	// mundur
	X_obj.push_back(-77.8); Y_obj.push_back(11.7);//1
	X_obj.push_back(-77.5); Y_obj.push_back(7.7);//2
	X_obj.push_back(-75.3); Y_obj.push_back(-1.8);//3
	X_obj.push_back(-70.8); Y_obj.push_back(-0.6);//4
	X_obj.push_back(-65.9); Y_obj.push_back(-0.5);// 5 titik terakhir
	X_obj.push_back(-49.68); Y_obj.push_back(-0.91);// 6
    
    // balik maju
    X_obj.push_back(-52.68); Y_obj.push_back(-0.91);//7ckpt
    X_obj.push_back(-61.9); Y_obj.push_back(-0.7);//8
    X_obj.push_back(-64.4); Y_obj.push_back(-0.6);//9
    X_obj.push_back(-98.1); Y_obj.push_back(-0.4);//10
    X_obj.push_back(-138.2); Y_obj.push_back(-0.3);//11
    
    //belok kiri PATAH
    X_obj.push_back(-139.1); Y_obj.push_back(0.1);//12
    X_obj.push_back(-142.0); Y_obj.push_back(-1.2);//13
    X_obj.push_back(-143.8); Y_obj.push_back(-4.5);//14
    X_obj.push_back(-144.4); Y_obj.push_back(-7.0);//15
    X_obj.push_back(-144.4); Y_obj.push_back(-10.1);//16

    
    //lurus 
    X_obj.push_back(-145.5); Y_obj.push_back(-47.9);//17
    X_obj.push_back(-145.5); Y_obj.push_back(-101.5);//18
    X_obj.push_back(-145.5); Y_obj.push_back(-108.6);//19
    
    //belok kiri
    X_obj.push_back(-144.6); Y_obj.push_back(-114.0);//20
    X_obj.push_back(-143.1); Y_obj.push_back(-118.8);//21
    X_obj.push_back(-139.5); Y_obj.push_back(-124.7);//22
    X_obj.push_back(-134.9); Y_obj.push_back(-129.8);//23
    X_obj.push_back(-130.0); Y_obj.push_back(-131.6);//24
    X_obj.push_back(-125.4); Y_obj.push_back(-132.8);//25
    X_obj.push_back(-120.9); Y_obj.push_back(-133.1);//26
    
    //lurus sblm belok
    X_obj.push_back(-85.3); Y_obj.push_back(-132.4);//27
    
    //belok kanan dan kiri
    X_obj.push_back(-77.2); Y_obj.push_back(-136.5);//28
    
    X_obj.push_back(-76.2); Y_obj.push_back(-143.3);//29
    X_obj.push_back(-75.0); Y_obj.push_back(-148.2);//30
    X_obj.push_back(-74.7); Y_obj.push_back(-160.9);//31
    
    X_obj.push_back(-72.6); Y_obj.push_back(-169.5);//32
    X_obj.push_back(-69.8); Y_obj.push_back(-176.1);//33
    X_obj.push_back(-65.8); Y_obj.push_back(-182.0);
    X_obj.push_back(-60.5); Y_obj.push_back(-187.2);
    X_obj.push_back(-54.9); Y_obj.push_back(-190.6);
    X_obj.push_back(-48.9); Y_obj.push_back(-192.8);
    
    //lurus
    X_obj.push_back(-41.6); Y_obj.push_back(-193.6);
    X_obj.push_back(-13.6); Y_obj.push_back(-193.9);
    
    //belok kiri POMBENSIN (NO NOTIF)
    X_obj.push_back(-10.1); Y_obj.push_back(-192.2);
    X_obj.push_back(-9.4); Y_obj.push_back(-189.4);
    X_obj.push_back(-9.4); Y_obj.push_back(-185.9);
    
    //lurus MASIH CACAT (NO NOTIF)
    X_obj.push_back(-9.4); Y_obj.push_back(-141.0);
    //kanan
    X_obj.push_back(-7.0); Y_obj.push_back(-134.8);
    X_obj.push_back(-0.5); Y_obj.push_back(-132.6);
    X_obj.push_back(10.1); Y_obj.push_back(-131.1);
    //lurus
    X_obj.push_back(16.5); Y_obj.push_back(-130.7);
    X_obj.push_back(131.5); Y_obj.push_back(-129.3);
    
    //kiri
    X_obj.push_back(138.8); Y_obj.push_back(-128.0);
    X_obj.push_back(142.6); Y_obj.push_back(-126.9);
    X_obj.push_back(146.4); Y_obj.push_back(-125.2);
    X_obj.push_back(151.0); Y_obj.push_back(-122.5);
    X_obj.push_back(155.3); Y_obj.push_back(-119.0);
    X_obj.push_back(159.4); Y_obj.push_back(-114.5);
    X_obj.push_back(162.1); Y_obj.push_back(-110.6);
    X_obj.push_back(164.4); Y_obj.push_back(-105.6);
    X_obj.push_back(166.2); Y_obj.push_back(-99.9);
    X_obj.push_back(166.9); Y_obj.push_back(-95.1);
    X_obj.push_back(167.1); Y_obj.push_back(-89.5);
    
    
    //lurus
    X_obj.push_back(167.4); Y_obj.push_back(-70.9);
    
    //kanan
    X_obj.push_back(172.8); Y_obj.push_back(-61.8);
    X_obj.push_back(179.0); Y_obj.push_back(-59.4);
    X_obj.push_back(184.6); Y_obj.push_back(-58.9);
    
    //lurus
    X_obj.push_back(220.6); Y_obj.push_back(-58.5);
    
    //kiri 
    X_obj.push_back(226.7); Y_obj.push_back(-55.3);
    X_obj.push_back(229.0); Y_obj.push_back(-50.2);
    X_obj.push_back(230.4); Y_obj.push_back(-43.2);
    
    //lurus
    X_obj.push_back(231.2); Y_obj.push_back(-20.0);
    
    //kiri
    X_obj.push_back(228.8); Y_obj.push_back(-14.7);
    X_obj.push_back(224.4); Y_obj.push_back(-11.9);
    X_obj.push_back(220.2); Y_obj.push_back(-10.5);
    X_obj.push_back(216.3); Y_obj.push_back(-10.1);
    
    
    //lurus
    X_obj.push_back(203.9); Y_obj.push_back(-9.4);
    X_obj.push_back(181.8); Y_obj.push_back(-9.2);
    X_obj.push_back(139.8); Y_obj.push_back(-8.6);
    X_obj.push_back(28.7); Y_obj.push_back(-7.0);
    
    //bunderan baru
    X_obj.push_back(23.5); Y_obj.push_back(-8.9);
    X_obj.push_back(20.3); Y_obj.push_back(-12.7);
    X_obj.push_back(15.0); Y_obj.push_back(-18.3);
    X_obj.push_back(9.6); Y_obj.push_back(-21.4);
    X_obj.push_back(1.8); Y_obj.push_back(-23.6);
    X_obj.push_back(-6.5); Y_obj.push_back(-22.9);
    X_obj.push_back(-13.5); Y_obj.push_back(-19.7);
    X_obj.push_back(-19.3); Y_obj.push_back(-13.9);
    X_obj.push_back(-22.5); Y_obj.push_back(-8.0);
    X_obj.push_back(-23.8); Y_obj.push_back(-0.9);
    X_obj.push_back(-22.8); Y_obj.push_back(6.6);
    X_obj.push_back(-19.9); Y_obj.push_back(12.3);
    // bunderan keluar
    X_obj.push_back(-15.9); Y_obj.push_back(16.6);
    X_obj.push_back(-11.3); Y_obj.push_back(22.0);
    X_obj.push_back(-8.5); Y_obj.push_back(27.0);
    X_obj.push_back(-7.7); Y_obj.push_back(30.4);
    X_obj.push_back(-6.9); Y_obj.push_back(35.2);
    X_obj.push_back(-6.3); Y_obj.push_back(41.8);
    
	//lurus
	X_obj.push_back(-6.3); Y_obj.push_back(45.6); 
	X_obj.push_back(-4.5); Y_obj.push_back(124.7);
	X_obj.push_back(-3.7); Y_obj.push_back(148.8);
	X_obj.push_back(-2.7); Y_obj.push_back(184.5);
	X_obj.push_back(-4.4); Y_obj.push_back(190.0); 
	
	//belok kiri 
	X_obj.push_back(-8.2); Y_obj.push_back(192.1);
	X_obj.push_back(-12.4); Y_obj.push_back(193.2);
	X_obj.push_back(-16.1); Y_obj.push_back(194.0);  
	
	//lurus  
    X_obj.push_back(-15.45); Y_obj.push_back(194.0);
    X_obj.push_back(-40.45); Y_obj.push_back(194.2);

}


double formula_teta(double x, double y) {
	
    double teta = std::atan2(y,x);
    
    teta *= -1;
    if (teta < 0) {
        teta += 6.283185307179586; // pi/2
    }
    return teta;
}

void speedometerCallback (const std_msgs::Float32::ConstPtr& msg) {
    if (msg->data >= 30/3.6) {
        Throttle_X = 0.465;
    }
    throttle(Throttle_X);
    
    if (abs(msg->data)<0.0005){
		brake(0.0);
		
}
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    double x_1 = msg->pose.pose.position.x;
    double y_1 = msg->pose.pose.position.y;
    
    if (x_1 == x_0 && y_1 == y_0) {
        return;
    }
    
	double jarak = sqrt((X_obj[count] - x_1) * (X_obj[count] - x_1) + (Y_obj[count] - y_1) * (Y_obj[count] - y_1));
    if (jarak <= 6.25) {
        count++;
    }

//x    
    double deltaX1 = x_1 - x_0;
    double deltaX2 = X_obj[count] - x_1;
//y    
    double deltaY1 = y_1 - y_0;
    double deltaY2 = Y_obj[count] - y_1;
    
//sudut    
    double alpha_1 = formula_teta(deltaX1, deltaY1);
    double alpha_2 = formula_teta(deltaX2, deltaY2);
    double beta = alpha_2 - alpha_1;
    
   
    x_0 = x_1;
    y_0 = y_1;
    

    if (beta > 3.14159265358979323846) { 
        beta -= 6.283185307179586; // 2pi
    }
    if (beta <= -3.14159265358979323846) {
        beta += 6.283185307179586; // 2pi
    }
    
    Steering_X = beta / 1.5707963267948966; // pi/2
    if (Steering_X < -1.0) {
        Steering_X = -1.0;
    }
    if (Steering_X > 1.0) {
        Steering_X = 1.0;
    }
    

    setCommand();
}

void lane_invasionCallback (const carla_msgs::CarlaLaneInvasionEvent::ConstPtr& msg) {
    for (auto x : msg->crossed_lane_markings) {
        std::cout<<x<<std::endl;
        std::cout<<" Fail: "<<"("<<x_0<<", "<<y_0<<")"<<std::endl;
        }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "shell_simulation_node");
    ros::NodeHandle nh;
    throttlePublisher = nh.advertise<std_msgs::Float64>("throttle_command", 1);
    brakePublisher = nh.advertise<std_msgs::Float64>("brake_command", 1);
    gearPublisher = nh.advertise<std_msgs::String>("gear_command", 1);
    steeringPublisher = nh.advertise<std_msgs::Float64>("steering_command", 1);
    initTarget();
    ros::Duration(1.0).sleep();
    odomSubscriber = nh.subscribe("carla/ego_vehicle/odometry", 1, odomCallback);
    speedometerSubscriber = nh.subscribe("carla/ego_vehicle/speedometer", 1 , speedometerCallback);
    lane_invasionSubscriber = nh.subscribe("carla/ego_vehicle/lane_invasion", 1, lane_invasionCallback);
    ros::spin();
}
