#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

float Kp = 100; //80 bien con vel-max 1
float Ki = 70; // 20 sin velocidad
float Kd = 0.9; //0.2
float Kpvel = 0.005; // bueno 0.0125
float Kivel = 0.01;
float Kdvel = 0.00;    // bueno 0.01

float angulo_actual = 0.0, angulo_error = 0.0, angulo_error_anterior = 0.0, suma_error_angulo = 0.0;
float angulo_equilibrio = 0.0, angulo_transitorio = 0.0;
float tiempo_equilibrio = 0.0, tiempo_transitorio = 0.0;
float velocidad_deseada = 0.0, velocidad_enviada = 0.0;
float velocidad_actual = 0.0, velocidad_error = 0.0, velocidad_error_anterior = 0.0, suma_velocidad_error = 0.0;
float angulo_giro=0.0;

ros::Duration t_dif;

void leer_imu(const sensor_msgs::ImuPtr mensaje)
{
	angulo_actual = mensaje->orientation.y;
	//ahora = mensaje->header.stamp;
}
void leer_odom(const nav_msgs::OdometryPtr mensaje)
{
	velocidad_actual = mensaje->twist.twist.linear.x;
}
void leer_velocidad(const geometry_msgs::TwistPtr velocidad)
{
	velocidad_deseada = velocidad->linear.x;
	angulo_giro = velocidad->angular.z;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "equilibrio");
	ros::NodeHandle n;

	ros::Subscriber imu_sub=n.subscribe("imu", 1, &leer_imu);
	ros::Subscriber odom_sub=n.subscribe("odom", 1, &leer_odom);
	ros::Subscriber vel_sub=n.subscribe("cmd_vel_deseada", 1, &leer_velocidad);
	ros::Publisher cmd_pub=n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	ros::Publisher control_pub=n.advertise<geometry_msgs::Twist>("control", 1);
	
	geometry_msgs::Twist velocidad;
	geometry_msgs::Twist control;
	
        ros::Time ahora = ros::Time::now();
	while (ahora.isZero()) ahora = ros::Time::now(); // Por ser tiempo simulado	
	ros::Time t_ultimo = ros::Time::now();
	
	float PID_Kp;
	float PID_Kpvel;
	float PID_Ki;
	float PID_Kivel;
	float PID_Kd;
	float PID_Kdvel;
	int A=0, B=0;
	
	while (ros::ok)
	{
	  if (angulo_actual < 0.35 and angulo_actual > -0.35)
	    {
	    
	     ROS_INFO("Ang Equil.: %f", angulo_equilibrio);
	     //ROS_INFO("A-B: %d-%d", A, B);
	    
	     ros::spinOnce();
	     ahora = ros::Time::now();
	     t_dif = ahora - t_ultimo;
	     t_ultimo = ahora;
	     
		if (t_dif.toSec() >= 0.001)
		   {
		     angulo_error = angulo_equilibrio - angulo_actual;
		
		     PID_Kp = Kp * angulo_error; // Kp
		
		     suma_error_angulo = suma_error_angulo + angulo_error * t_dif.toSec();
		
		     PID_Ki = Ki * suma_error_angulo; // Ki
		     
		     PID_Kd = Kd * (angulo_error - angulo_error_anterior) / t_dif.toSec();
		     if (isnan(PID_Kd)) PID_Kd = 0;
		     
		     velocidad_enviada = - PID_Kp - PID_Ki + PID_Kd;
		     if (velocidad_enviada > 0.5) velocidad_enviada = 0.5;   // limite segun motor
		     if (velocidad_enviada < -0.5) velocidad_enviada = -0.5;
		
		     velocidad.linear.x = velocidad_enviada;
		     velocidad.linear.y = 0;
		     velocidad.angular.z = angulo_giro;
		     
		     control.linear.x = PID_Kp;
		     control.linear.y = PID_Ki;
		     control.linear.z = PID_Kd;
		     control.angular.x = PID_Kpvel;
		     control.angular.y = angulo_equilibrio * 1000;
		     control.angular.z = angulo_actual * 1000;
		     
		     cmd_pub.publish(velocidad);
		     control_pub.publish(control);
		     //ROS_INFO("Ang-Eq-Act %f:%f:%f", angulo_equilibrio, angulo_actual, tiempo_equilibrio);
		
		     angulo_error_anterior = angulo_error;
		     A += 1;
		    }
		    
		 if (t_dif.toSec() >= 0.002)
		    {
		      ros::spinOnce();
		      velocidad_error = velocidad_deseada - velocidad_actual;
		      PID_Kpvel = Kpvel * velocidad_error;
		      suma_velocidad_error = suma_velocidad_error + velocidad_error * t_dif.toSec();
		      PID_Kivel = Kivel * suma_velocidad_error;
		      
		      PID_Kdvel = Kdvel * (velocidad_error - velocidad_error_anterior) / t_dif.toSec();
		      
		      angulo_equilibrio =  PID_Kpvel + PID_Kivel + Kdvel;
		      		      
		      velocidad_error_anterior = velocidad_error;
		      B += 1;
		    }
	   }
	    else {
	           velocidad.linear.x = 0;
	           velocidad.angular.z = 0;
		   cmd_pub.publish(velocidad);
		 }
	}
}
