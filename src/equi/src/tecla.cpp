#include "ros/ros.h"
#include <sys/ioctl.h>
#include <termios.h>
//#include <stdio.h>
#include <cstdio>



int main(int argc, char **argv)
{
ros::init(argc, argv, "equilibrio");
ros::NodeHandle n;

char c=0;
struct termios param_save, misparam;

/* -- lectura de parametros actuales de dispositivo --- */
ioctl(0, TCGETA, &param_save);

/* --- programacion del dispositivo --- */
misparam = param_save;
/* quitamos eco local y modo canonico */
misparam.c_lflag &= ~(ICANON | ECHO);
/* devolver entrada cuando se halle 1 caracter en buffer */
misparam.c_cc[4]=1;
ioctl(0, TCSETA, &misparam ); /* pone parametros */

while(ros::ok)
  {int n;
   read(0, &c, 1);
   ROS_INFO("tecla : %d(%d) ", c, n);
   n += 1;
  }
  
ioctl(0, TCSETA, &param_save );
}

