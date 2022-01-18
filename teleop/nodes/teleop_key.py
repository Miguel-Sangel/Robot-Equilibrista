#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

msg = """
Control por teclado
---------------------------
     W:  ADELANTE
     X:  ATRAS
     A:  IZQUIERDA
     D:  DERECHA
     S:  ANULAR GIRO
     
     ESPACIO PARAR

"""

e = """
Communications Failed
"""

def getKey():
    if os.name == 'nt':
      return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(target_linear_vel, target_angular_vel):
    return "Actualmente:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

def constrain(input, max, min):
    if input < min:
      input = min
    elif input > max:
      input = max    
    else:
      input = input

    return input


if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop')
    pub = rospy.Publisher('cmd_vel_deseada', Twist, queue_size=10)

    target_linear_vel   = -4.5
    target_angular_vel  = 0.0
    twist = Twist()
    twist.linear.x = -4.5; twist.linear.y = 0.0; twist.linear.z = 0.0
    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0


    try:
        print(msg)
        while(1):
            key = getKey()
            if key == 'w' :
                target_linear_vel = constrain(target_linear_vel + 0.5, 25, -35)
##                if target_linear_vel > -51 and target_linear_vel < 0:
##                      target_linear_vel = 0
##                if target_linear_vel == 1:
##                      target_linear_vel = 51 
            elif key == 'x' :
                target_linear_vel = constrain(target_linear_vel - 0.5,  25, -35)
##                if target_linear_vel < 51 and target_linear_vel > 0:
##                      target_linear_vel = 0
##                if target_linear_vel == -1:
##                      target_linear_vel = -51 
            elif key == 'a' :
                target_angular_vel = constrain(target_angular_vel + 5, 50, -50)

            elif key == 'd' :
                target_angular_vel = constrain(target_angular_vel - 5, 50, -50)

            elif key == 's' :
                if target_angular_vel == 0.0 :
                     target_linear_vel   = -4.5
                else:
                    target_angular_vel  = 0.0
            elif key == ' ':
                 target_linear_vel = 1000.0
                 target_angular_vel  = 0.0
                 twist.linear.x = target_linear_vel
                 twist.angular.z = target_angular_vel
                 pub.publish(twist)
                 target_linear_vel = -4.5
                 twist.linear.x = target_linear_vel
            else:
                if (key == '\x03'):
                    break

            print(key)
##            if status == 20 :
            print(msg)
            print("velocidad Linear %s" % target_linear_vel)
            print("velocidad Angular %s" % target_angular_vel)

            if twist.linear.x != target_linear_vel or twist.angular.z != target_angular_vel:
                 twist.linear.x = target_linear_vel
                 twist.angular.z = target_angular_vel
                 pub.publish(twist)

    except:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
