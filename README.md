# Robot-Equilibrista
 
Este proyecto esta enfocado en crear un robot con una base de dos ruedas con control diferencial y que sea capaz de mantener el equilibrio.
Proyectos de este tipo se pueden desarrollar de distintas formas y de hecho se pueden encontrar unos cuantos ejemplos aquí en GitHub, pero la particularidad de este proyecto es hacerlo bajo el entorno de programación de robot ROS.

# Descripción
Los componentes utilizados para la construcción de este robot son:

- Raspberry Pi 3
- 2 motores GA37-520 DC de 12V. Con Encoder incorporado
- Controlador para los motores TB6612FNG
- Sensor IMU MPU-6050 (acelerómetro 3 ejes y giroscopio 3 ejes)
- Bateria
- Camara USB
- Carcasa realizada en impresión 3D por el Club de Robótica de Granada.

Esquema de conexiones. (Los puertos GPIO de la Raspberry se pueden cambiar a su gusto siempre que el codigo se reajuste para que concuerde con los valores escogidos)

![Esquema (Encoder antiguo)](https://user-images.githubusercontent.com/97805074/149966309-c9ce76df-25f0-4d9f-94e3-49a38e96fd82.jpg)

# Instalación
Todo el proceso descrito a continuación esta orientado al sistema operativo Ubuntu Mate 18.04 LTS, que es el que admite la Raspberry  pi 3 Model B+ y el framework ROS. Para versiones posteriores tener en cuenta las indicaciones de los sitios oficiales.

En primer lugar instalaremos la biblioteca Pigpio para poder acceder a los puertos GPIO  de la Raspberry. Para esto seguiremos las indicaciones del sitio: https://abyz.me.uk/rpi/pigpio/download.html

Iniciaremos una terminal y ejecutaremos esta serie de instrucciones:

 - wget https://github.com/joan2937/pigpio/archive/master.zip
 - unzip master.zip
 - cd pigpio-master
 - make
 - sudo make install

Instalación del Sistema Operativo para Robot ROS. 
Teniendo en cuenta que la versión de ROS depende de la versión de Ubuntu, en nuestro caso la versión 18.04, eligiéremos ROS-Kinetic. http://wiki.ros.org/kinetic/Installation/Ubuntu

Abriendo una terminal de Ubuntu teclearemos las siguientes instrucciones:

	- sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.lis
	- sudo apt install curl # if you haven't already installed curl
	- curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
	- sudo apt-get update
	- sudo apt-get install ros-kinetic-desktop-full
 
 - echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
 - source ~/.bashrc

Con esto tendremos instalado ROS, ahora instalaremos otros paquetes que nos ampliaran las capacidades de ROS.

Rosbridge_suit, http://wiki.ros.org/rosbridge_suite, es una herramienta que nos permite conectar con el sistema ROS del robot a traves de una pagina web.

 - sudo apt-get install ros-kinetic-rosbridge-server

Usb_cam, http://wiki.ros.org/usb_cam, paquete que permite el acceso de ROS a los datos devueltos por la Camara USB.

 - sudo apt-get install ros-kinetic-usb-cam
 
