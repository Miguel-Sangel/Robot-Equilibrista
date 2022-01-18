#include "ros/ros.h"
#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <sensor_msgs/Imu.h>

#define IMU_ID 0x68 // IMU MPU6050
#define PWR_MGMT_1   0x6B
#define SMPLRT_DIV   0x19
#define CONFIG       0x1A
#define GYRO_CONFIG  0x1B
#define INT_ENABLE   0x38
#define ACCEL_XOUT_H 0x3B
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F
#define GYRO_XOUT_H  0x43
#define GYRO_YOUT_H  0x45
#define GYRO_ZOUT_H  0x47

int fd;

short leo_datos_registro(int addr){
	short high_byte,low_byte,value;
	high_byte = wiringPiI2CReadReg8(fd, addr);
	low_byte = wiringPiI2CReadReg8(fd, addr+1);
	value = (high_byte << 8) | low_byte;
	return value;
}



int main(int argc, char **argv)
{  ros::init(argc, argv, "imu");
   ros::NodeHandle nh;
   
   // Setup I2C communication
   fd = wiringPiI2CSetup(IMU_ID);
   if (fd == -1) {
        ROS_ERROR("Fallo en I2C communication.\n");
        return -1;
    }
    ROS_INFO("I2C setup inicicado\n");

    wiringPiI2CWriteReg8 (fd, SMPLRT_DIV, 0x07);/* divisor para el calculo velocidad muestreo del sensor 8Khz / 8=(0x07) = 1Khz*/
	wiringPiI2CWriteReg8 (fd, PWR_MGMT_1, 0x01);/* eje X giroscopio como referencia reloj*/
	wiringPiI2CWriteReg8 (fd, CONFIG, 0);		/* muestreo giroscopio 8Khz */
	wiringPiI2CWriteReg8 (fd, GYRO_CONFIG, 0);  /* escala Gyro +/- 250 º/s */
	wiringPiI2CWriteReg8 (fd, INT_ENABLE, 0x01);/* Habilita interupcion datos listos */
    
    ros::Publisher IMU_pub = nh.advertise<sensor_msgs::Imu>("mimu", 1);
    sensor_msgs::Imu mensaje;

    float acel_x, acel_y, acel_z;
    float giro_x, giro_y, giro_z;
    float acel_angulo_y, giro_angulo_y, angulo_y, angulo_ant_y;
    ros::Time ahora, t_ant = ros::Time::now();
    ros::Duration dt;

    while (ros::isShuttingDown) {
        ahora = ros::Time::now();
        dt = ahora - t_ant;
        t_ant = ahora;
        acel_x = leo_datos_registro(ACCEL_XOUT_H);
        acel_y = leo_datos_registro(ACCEL_YOUT_H);
        acel_z = leo_datos_registro(ACCEL_ZOUT_H);
        giro_y = leo_datos_registro(GYRO_YOUT_H);

        acel_angulo_y = atan(-acel_x/sqrt(pow(acel_y, 2) + pow(acel_z, 2))) * (180.0 / 3.14);
        giro_angulo_y = angulo_ant_y + (giro_y / 131) * dt.toSec();
        angulo_y = 0.95 * giro_angulo_y + 0.05 * acel_angulo_y;
        angulo_ant_y = angulo_y;

        mensaje.header.stamp = ahora;
        mensaje.orientation.y = angulo_y;
        mensaje.orientation.z = acel_angulo_y;
        mensaje.orientation.x = giro_angulo_y;

        IMU_pub.publish(mensaje);
        ROS_INFO("Angulo eje -y- : %f", angulo_y); 
    }
    close (fd);
}
