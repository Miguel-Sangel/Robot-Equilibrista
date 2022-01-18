#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

float angulo_actual, angulo_dif;
float angulo_equilibrio;
float velocidad_deseada=0.0;
float velocidad_momento=0.0;
double angulo_giro=0.0;
ros::Time ahora, ultimo;

void leer_imu(const sensor_msgs::ImuPtr mensaje)
{
	angulo_actual = mensaje->orientation.y;
	ahora = mensaje->header.stamp;
}
void leer_odom(const nav_msgs::OdometryPtr mensaje)
{
	velocidad_momento = mensaje->twist.twist.linear.x;
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "equilibrio");
	ros::NodeHandle n;

	ros::Subscriber imu_sub=n.subscribe("imu", 1, &leer_imu);
	ros::Subscriber odom_sub=n.subscribe("odom", 1, &leer_odom);
	ros::Publisher cmd_pub=n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	
	geometry_msgs::Twist velocidad;
	
	while (ros::ok)
	{
		ros::spinOnce();
		if (angulo_actual < 0.40 and angulo_actual > -0.40)
		{
			angulo_dif = angulo_equilibrio - angulo_actual;
			ROS_WARN("A-C= %f:%f", velocidad_momento, velocidad.linear.x);
			if (angulo_dif < 0)
				velocidad.linear.x =  angulo_dif * angulo_dif * angulo_dif * angulo_dif * 1500000;
			else
				velocidad.linear.x =  angulo_dif * angulo_dif * angulo_dif * angulo_dif * -1500000;
			velocidad.angular.z = angulo_giro;
			cmd_pub.publish(velocidad);
			ROS_INFO("A-C= %f:%f", angulo_dif, velocidad.linear.x);
		}
		else {
			velocidad.linear.x = 0;
			cmd_pub.publish(velocidad);
		     }
	}
}
