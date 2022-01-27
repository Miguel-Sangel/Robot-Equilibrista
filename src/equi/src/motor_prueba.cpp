#include "ros/ros.h"
//#include <wiringPi.h>
//#include <softPwm.h>
#include <pigpio.h>

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include <sensor_msgs/Imu.h>

//----------VARIABLES MOTOR ----------------------------
#define velocidad_izquierdo 25
#define velocidad_derecho 19
#define Adelante_izquierdo 23
#define Atras_izquierdo 24
#define Adelante_derecho 27
#define Atras_derecho 5
#define stby 21

int velocidad_deseada = 0, angulo_giro = 0;
bool motor_parado_dcho = true, motor_parado_izdo = true;
bool stop = false;

void leer_velocidad_deseada(const geometry_msgs::TwistPtr velocidad)
{
	velocidad_deseada = velocidad->linear.x;
	angulo_giro = velocidad->angular.z;
    if (velocidad_deseada > 999) { stop = false; 
                                   velocidad_deseada = 0;
                                   ROS_INFO("ESTOP : %d", stop); }
}
//------------------------------------------------------

//----------VARIABLES Y FUNCIONES ENCODER ---------------
#define sentido_izquierdo 12
#define sentido_derecho 13

#define paso_izquierdo 16
#define paso_derecho 18

int suma_paso_izdo = 0, suma_paso_dcho = 0;
int sent_izdo = 0, sent_dcho = 0;

float vel_actual_dcho = 0, vel_actual_izdo = 0;


void func_pasos_izdo(int gpio, int nivel, uint32_t tiempo)
{   if (nivel == RISING_EDGE) {        
        if (sent_izdo == 0)
            suma_paso_izdo -= 1;
        else if (sent_izdo == 1)
            suma_paso_izdo += 1;
        ROS_WARN("suma_paso_IZDO : %d", suma_paso_izdo);
       }
}

void func_pasos_dcho(int gpio, int nivel, uint32_t tiempo)
{   if (nivel == RISING_EDGE) {
        if (sent_dcho == 0)
            suma_paso_dcho += 1;
        else if (sent_dcho == 1)
            suma_paso_dcho -= 1;
       ROS_WARN("suma_paso_DRCHO : %d", suma_paso_dcho);
       }
}

void func_sentido_izdo(int gpio, int nivel, uint32_t tiempo)
{
    if (nivel == 0) sent_izdo = 0;
    else if (nivel == 1) sent_izdo = 1;
    else sent_izdo = 2;
   // ROS_INFO("sentido : %d", cont_sent_izdo); 
}

void func_sentido_dcho(int gpio, int nivel, uint32_t tiempo)
{
    if (nivel == 0) sent_dcho = 0;
    else if (nivel == 1) sent_dcho = 1;
    else sent_dcho = 2;
    //ROS_INFO("sentido : %d", sent_dcho); 
}
//----------------------------------------------------------
//-----------------------------------------------------------


int main(int argc, char **argv)
{   //setenv("WIRINGPI_GPIOMEM", "1", 1); // wiringpi como root
	ros::init(argc, argv, "motor_prueba");
	ros::NodeHandle n;

    

    if (gpioInitialise() < 0) return 1;

    //---------------MOTOR-----------------------
    gpioSetMode(Adelante_izquierdo, PI_OUTPUT);
    gpioSetMode(Atras_izquierdo, PI_OUTPUT);
    gpioSetMode(Adelante_derecho, PI_OUTPUT);
    gpioSetMode(Atras_derecho, PI_OUTPUT);
    gpioSetMode(stby, PI_OUTPUT);

    gpioSetPullUpDown(Adelante_izquierdo, PI_PUD_UP);
    gpioSetPullUpDown(Atras_izquierdo, PI_PUD_UP);
    gpioSetPullUpDown(Adelante_derecho, PI_PUD_UP);
    gpioSetPullUpDown(Atras_derecho, PI_PUD_UP);
    gpioSetPullUpDown(stby, PI_PUD_UP);

    gpioSetMode(velocidad_izquierdo, PI_OUTPUT);
    gpioSetMode(velocidad_derecho, PI_OUTPUT);
    gpioPWM(velocidad_izquierdo, 0);
    gpioPWM(velocidad_derecho, 0);
    //gpioSetPWMfrequency(velocidad_izquierdo, 500);
    //gpioSetPWMfrequency(velocidad_derecho, 500);
    gpioSetPWMrange(velocidad_izquierdo, 255);
    gpioSetPWMrange(velocidad_derecho, 255);
    
    //----------------ENCODER-------------------
    gpioSetMode(sentido_izquierdo, PI_INPUT);
    gpioSetMode(sentido_derecho, PI_INPUT);
    gpioSetMode(paso_izquierdo, PI_INPUT);
    gpioSetMode(paso_derecho, PI_INPUT);

    gpioSetPullUpDown(sentido_izquierdo, PI_PUD_DOWN);
    gpioSetPullUpDown(paso_izquierdo, PI_PUD_DOWN);
    gpioSetPullUpDown(sentido_derecho, PI_PUD_DOWN);
    gpioSetPullUpDown(paso_derecho, PI_PUD_DOWN);
         //----- interupciones -----
    gpioSetAlertFunc(paso_derecho, func_pasos_dcho);
    gpioSetAlertFunc(paso_izquierdo, func_pasos_izdo);
    gpioSetAlertFunc(sentido_derecho, func_sentido_dcho);
    gpioSetAlertFunc(sentido_izquierdo, func_sentido_izdo);
    //------------------------------------------
	
    ros::Subscriber vel_sub=n.subscribe("cmd_vel_deseada", 1, &leer_velocidad_deseada);
    ros::Publisher control_pub=n.advertise<geometry_msgs::Twist>("control", 1);
    geometry_msgs::Twist control;

    double pwm_left = 0;                // ¿sentido y salida pwm?
    double pwm_right = 0;
	velocidad_deseada = 0;	

	while (ros::ok)
    {
      ros::spinOnce();
      if (velocidad_deseada != 0) gpioWrite(stby, 1);
      else { gpioWrite(stby, 0);
             suma_paso_izdo = 0;
             suma_paso_dcho = 0;
           }
      pwm_left = velocidad_deseada;
      pwm_right = velocidad_deseada;

      if (pwm_left < -255) pwm_left = -255;
      else if (pwm_left > 255) pwm_left = 255;
      if (pwm_right < -255) pwm_right = -255;
      else if (pwm_right > 255) pwm_right = 255;

      if (pwm_left < 0)
         { gpioWrite(Adelante_izquierdo, 0);
           gpioWrite(Atras_izquierdo, 1);
           gpioPWM(velocidad_izquierdo, -pwm_left);
         }
      else { gpioWrite(Adelante_izquierdo, 1);
             gpioWrite(Atras_izquierdo, 0);
             gpioPWM(velocidad_izquierdo, pwm_left);
           }
      if (pwm_right < 0)
         { gpioWrite(Adelante_derecho, 0);
           gpioWrite(Atras_derecho, 1);
           gpioPWM(velocidad_derecho, -pwm_right);
         }
      else  { gpioWrite(Adelante_derecho, 1);
              gpioWrite(Atras_derecho, 0);
              gpioPWM(velocidad_derecho, pwm_right);
            }
     }
}
