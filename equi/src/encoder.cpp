#include <ros/ros.h>
#include <wiringPi.h>
#include <geometry_msgs/Twist.h>

#define Drch_Velo 22
#define Drch_Giro 23

static volatile int contador_Drch = 0;

void suma_Drch() {
   contador_Drch = +1;
}

int main(int argc, char **argv) {

   ros::init(argc, argv, "encoder");
   ros::NodeHandle nodo;
   ros::Publisher velocidad=nodo.advertise<geometry_msgs::Twist>("odom", 1);

   wiringPiSetup();

   wiringPiISR(Drch_Velo, INT_EDGE_RISING, &suma_Drch);


}
