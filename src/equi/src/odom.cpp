#include "ros/ros.h"
//#include <wiringPi.h>
//#include <softPwm.h>
#include <pigpio.h>

#include "geometry_msgs/TwistStamped.h"


#define sentido_izquierdo 18
#define sentido_derecho 13

#define paso_izquierdo 12
#define paso_derecho 26
// relacion ejes 45:1, diametro 0.65,- Longitud = 0.065 * 3.14 = 0.204198
// longitud por vuelta motor = 0.204198 / 45 = 0.004537
// media vuelta = 0.004537 / 2 = 0.00226
#define long_med_vuelt_eje_motor 0.0022688727


int cont_paso_izq = 0, cont_paso_dcho = 0;
int cont_sent_izq = 0, cont_sent_dcho = 0;
uint32_t starTick, sumaTick = 0;

//ros::Time t_actual, t_anterior = ros::Time::now();
geometry_msgs::TwistStamped velocidad;

void func_pasos_derecho(int gpio, int nivel, uint32_t tiempo)
{
    cont_paso_dcho += 1;
    sumaTick = sumaTick + tiempo;
    if ((cont_paso_dcho % 5) == 0){
         velocidad.header.stamp = ros::Time::now();
         velocidad.twist.linear.x = long_med_vuelt_eje_motor * sumaTick / 1000000;
       }
    ROS_INFO("pasos : %d", cont_paso_dcho);
}
void func_sentido_derecho(int gpio, int nivel, uint32_t tiempo)
{
    cont_sent_dcho += 1;
    ROS_INFO("sentido : %d", cont_sent_dcho);
}


int main(int argc, char **argv)
{   
	ros::init(argc, argv, "odom");
	ros::NodeHandle n;


    if (gpioInitialise() < 0) return 1;
    gpioSetMode(sentido_izquierdo, PI_INPUT);
    gpioSetMode(sentido_derecho, PI_INPUT);
    gpioSetMode(paso_izquierdo, PI_INPUT);
    gpioSetMode(paso_derecho, PI_INPUT);

    gpioSetAlertFunc(paso_derecho, func_pasos_derecho);
    gpioSetAlertFunc(sentido_derecho, func_sentido_derecho);
    

	
	ros::Publisher vel_pub=n.advertise<geometry_msgs::TwistStamped>("odom", 10);
    
    ros::Duration dt;
    starTick = gpioTick();
	
	while (ros::ok){
	  if (cont_paso_dcho == 40) {
          velocidad.twist.angular.z = 1;
          vel_pub.publish(velocidad);
        }      
      
      }
      
    ROS_INFO("SA CABO");
    //softPwmWrite(velocidad_izquierdo, 0);
    //softPwmWrite(velocidad_derecho, 0);
    gpioTerminate();
}
