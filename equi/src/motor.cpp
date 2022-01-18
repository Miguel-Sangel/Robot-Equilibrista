#include "ros/ros.h"
//#include <wiringPi.h>
//#include <softPwm.h>
#include <pigpio.h>
#include "KalmanFilter.h"
#include "geometry_msgs/Twist.h"

//----------VARIABLES MOTOR ----------------------------
#define velocidad_izquierdo 25
#define velocidad_derecho 19
#define Adelante_izquierdo 23
#define Atras_izquierdo 24
#define Adelante_derecho 27
#define Atras_derecho 5
#define stby 21
int velocidad_deseada = 0, angulo_giro = 0;
float angle_zero = -4.5;

bool stop = true;

void leer_velocidad_deseada(const geometry_msgs::TwistPtr velocidad)
{
    angle_zero = velocidad->linear.x;
    angulo_giro = velocidad->angular.z;
    if (angle_zero > 999) { stop = false; 
                            angle_zero = -4.5;
                            ROS_WARN("Ehhhhh Quieto");
                          }
}
//------------------------------------------------------

//----------VARIABLES Y FUNCIONES ENCODER ---------------
#define sentido_izquierdo 12
#define sentido_derecho 13

#define paso_izquierdo 16
#define paso_derecho 18
// relacion ejes 45:1, diametro 0.65,- Longitud = 0.065 * 3.14 = 0.204198
// longitud por vuelta motor = 0.204198 / 45 = 0.004537
// longitud por paso = 0.004537 / 11 = 0.000412455
// #define long_paso_motor 0.000412455

int suma_paso_izdo = 0, suma_paso_dcho = 0;
int sent_izdo = 0, sent_dcho = 0;

float vel_actual_dcho = 0, vel_actual_izdo = 0;


void func_pasos_izdo(int gpio, int nivel, uint32_t tiempo)
{   if (nivel == RISING_EDGE) {        
        if (sent_izdo == 0)
            suma_paso_izdo -= 1;
        else if (sent_izdo == 1)
            suma_paso_izdo += 1;
        //ROS_WARN("suma_paso_IZDO : %d", suma_paso_dcho);
       }
}

void func_pasos_dcho(int gpio, int nivel, uint32_t tiempo)
{   if (nivel == RISING_EDGE) {
        if (sent_dcho == 0)
            suma_paso_dcho += 1;
        else if (sent_dcho == 1)
            suma_paso_dcho -= 1;
       //ROS_WARN("suma_paso_DRCHO : %d", suma_paso_dcho);
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

//-------------------SENSOR IMU-----------------------------
#define IMU_ID 0x68 // IMU MPU6050
#define PWR_MGMT_1   0x6B
#define SMPLRT_DIV   0x19
#define CONFIG       0x1A
#define GYRO_CONFIG  0x1B
#define INT_ENABLE   0x38
#define ACCEL_XOUT_H 0x3B

int fd;

int16_t unir(short n) { return (n << 8) | (n >> 8) & 0xff; }
short lectura[7];

int16_t ax, ay, az, gx, gy, gz;
float dt = 0.005, Q_angle = 0.001, Q_gyro = 0.005, R_angle = 0.5, C_0 = 1, K1 = 0.05;

void inicia_imu(){
   fd = i2cOpen(1, IMU_ID, 0);
   i2cWriteByteData (fd, SMPLRT_DIV, 0x07);/* divisor para el calculo velocidad muestreo del sensor 8Khz / 8=(0x07) = 1Khz*/
   i2cWriteByteData (fd, PWR_MGMT_1, 0x01);/* eje X giroscopio como referencia reloj*/
   i2cWriteByteData (fd, CONFIG, 0);		/* muestreo giroscopio 8Khz */
   i2cWriteByteData (fd, GYRO_CONFIG, 0);  /* escala Gyro +/- 250 º/s */
   i2cWriteByteData (fd, INT_ENABLE, 0x01);/* Habilita interupcion datos listos */
}

void lee_imu() {
        
        i2cReadI2CBlockData(fd, ACCEL_XOUT_H, (char*)lectura, 14);
        ax = unir(lectura[0]);
        ay = unir(lectura[1]);
        az = unir(lectura[2]);
        gx = unir(lectura[4]);
        gy = unir(lectura[5]);
        gz = unir(lectura[6]);
   //ROS_WARN("imu : %f, %f, %f, %f,", ax, ay, az, gz);
}
//-----------------------------------------------------------

/////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{  	ros::init(argc, argv, "motor");
	ros::NodeHandle n;

    if (gpioInitialise() < 0) return 1;
    inicia_imu();
    KalmanFilter kalmanfilter;

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
	
    double kp_balance = 42, kd_balance = 0.7; // p = 55, 60, 80  --- d = 0.75,  0.85,  1,  2.5,
    double kp_speed = 10, ki_speed = 0.005;  // P = 10    --- i = 0.26
    double kp_turn = 2.5, kd_turn = 0.5;
    
    double pwm_left = 0;
    double pwm_right = 0;
    double speed_control_output = 0;
    double rotation_control_output = 0;
    double balance_control_output = 0;
    double speed = 0;
    double speed_filter = 0;
    double speed_filter_old = 0;
    double speed_integeral = 0;
    
    //Setting MPU6050 calibration parameters

    float angular_velocity_zero = 0.2; // - 5.0 x axle angular velocity calibration
    float angulo_actual = 0;

    ros::Time ahora, t_ultimo = ros::Time::now();
    ros::Duration t_dif_vel;
	
    stop = true;
    int contador = 0;

    while (ros::ok)
    {
      //ROS_WARN("Ang-Act : %f", angulo_actual);
      ros::spinOnce();  // obtengo velocidad_deseada y desbloquea stop
      ahora = ros::Time::now();
      if (!stop) 
      {  
         t_dif_vel = ahora - t_ultimo;
         dt = t_dif_vel.toSec();
         if (dt >= 0.005) 
            { t_ultimo = ahora;
              contador ++;
              lee_imu();
              kalmanfilter.Angle(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);
              angulo_actual = kalmanfilter.angle;
     //ROS_WARN("Angulo Actual : %f,", angulo_actual);
              if (angulo_actual > 25.0 or angulo_actual < -35.0)
                 { stop = true;
                   ROS_ERROR("Joder que me caigo"); 
                   kalmanfilter.Limpiar();
                   contador = 0;
                   suma_paso_dcho = 0;
                   suma_paso_izdo = 0;
                   pwm_left = 0;                // ¿sentido y salida pwm?
                   pwm_right = 0;
                   speed_control_output = 0;
                   rotation_control_output = 0;
                   balance_control_output = 0;
                   speed = 0;
                   speed_filter = 0;
                   speed_filter_old = 0;
                   speed_integeral = 0;
                   gpioPWM(velocidad_izquierdo, pwm_left);
                   gpioPWM(velocidad_derecho, pwm_right);
                   gpioWrite(stby, 0);
                 }
                 else { gpioWrite(stby, 1);
                        balance_control_output = kp_balance * (angulo_actual - angle_zero) + kd_balance * (kalmanfilter.Gyro_y - angular_velocity_zero);
                        if (contador == 8)
                          {  contador = 0;
                             speed = (suma_paso_dcho + suma_paso_izdo) * 0.5;
                             suma_paso_dcho = 0;
                             suma_paso_izdo = 0;
                             speed_filter = speed_filter_old * 0.7 + speed * 0.3; // filtro paso bsjoo
                             speed_filter_old = speed_filter;
                             speed_integeral += speed_filter; // velocidad acumulada
                             speed_integeral += -velocidad_deseada;   // velocidad acumulada descontando la deseada
                             if ( speed_integeral < -3000) speed_integeral = -3000;
                             else if ( speed_integeral > 3000) speed_integeral = 3000;
                             speed_control_output = kp_speed * speed_filter + ki_speed * speed_integeral;
                             rotation_control_output = angulo_giro - kd_turn * kalmanfilter.Gyro_z;
                            // ROS_WARN("Angulo_actual %f - Gyro_y %f", angulo_actual, kalmanfilter.Gyro_y);
                            // ROS_ERROR("pasos %f - speed %f - integral %f - control %f", speed, speed_filter, speed_integeral, speed_control_output);
                          }
                        pwm_left = balance_control_output + speed_control_output - rotation_control_output;
                        pwm_right = balance_control_output + speed_control_output + rotation_control_output;
                       // ROS_INFO("balance %f - speed %f - pwm %f", balance_control_output, speed_control_output, pwm_left);

                        if (pwm_left < -255) pwm_left = -255;
                        else if (pwm_left > 255) pwm_left = 255;
                        if (pwm_right < -255) pwm_right = -255;
                        else if (pwm_right > 255) pwm_right = 255;
                        //ROS_WARN("Rueda Drch %f - Rueda Izda %f", pwm_right, pwm_left);
              
                        if (pwm_left < 0)
                          { gpioWrite(Adelante_izquierdo, 1);
                            gpioWrite(Atras_izquierdo, 0);
                            gpioPWM(velocidad_izquierdo, -pwm_left);
                          }
                          else { gpioWrite(Adelante_izquierdo, 0);
                                 gpioWrite(Atras_izquierdo, 1);
                                 gpioPWM(velocidad_izquierdo, pwm_left);
                               }
                        if (pwm_right < 0)
                          { gpioWrite(Adelante_derecho, 1);
                            gpioWrite(Atras_derecho, 0);
                            gpioPWM(velocidad_derecho, -pwm_right);
                          }
                          else  { gpioWrite(Adelante_derecho, 0);
                                  gpioWrite(Atras_derecho, 1);
                                  gpioPWM(velocidad_derecho, pwm_right);
                                }
                    } // else  angulo_actual > 25
                 
	   } // dt >= 0.005
	 
      } else   // stop
             {  t_ultimo = ahora;
                kalmanfilter.Limpiar();
                contador = 0;
                suma_paso_dcho = 0;
                suma_paso_izdo = 0;
                pwm_left = 0;                // ¿sentido y salida pwm?
                pwm_right = 0;
                speed_control_output = 0;
                rotation_control_output = 0;
                balance_control_output = 0;
                speed = 0;
                speed_filter = 0;
                speed_filter_old = 0;
                speed_integeral = 0;
                gpioPWM(velocidad_izquierdo, pwm_left);
                gpioPWM(velocidad_derecho, pwm_right);
                gpioWrite(stby, 0);
             }
    } // while
    gpioWrite(stby, 0);
    gpioWrite(Adelante_izquierdo, 0);
    gpioWrite(Atras_izquierdo, 0);
    gpioWrite(Adelante_derecho, 0);
    gpioWrite(Atras_derecho, 0);
    gpioPWM(velocidad_izquierdo, 0);
    gpioPWM(velocidad_derecho, 0);
    ROS_INFO("SA CABO");
    gpioTerminate();
    i2cClose(fd);
}
