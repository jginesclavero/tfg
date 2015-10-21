/*
 * 
 * 
 * Autor: Jonathan Ginés Clavero (jonathangines@hotmail.com)
 * Fecha: 30/09/2015
 *  
 * Nodo para la navegación por VFF
 * 
 */

#include <ros/ros.h>
#include <tfg_jonathan_gines/Forces.h>
#include "std_msgs/String.h"
#include <sstream>
#include <math.h>
#include <list>
#include <iostream>
#include <kobuki_msgs/MotorPower.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

std::vector<float> forces(2,0);     // [0] modulo | [1] angulo
float v_prev_linear = 0.0;
float integral_linear[10];
float v_prev_angular = 0.0;
float integral_angular[10];
int cont = 0;


float
linear_PID(float module){
  int i;
  float add,v,dev=0.0;  
  v = ((10.0 - module)/10.0)*0.7;
  integral_linear[cont] = v;
  cont++;
  dev = v - v_prev_linear;
  v_prev_linear = v; 
  for (i=0;i<10;i++){
      add = add + integral_linear[i];
  }
  return (0.8* v + 0.1 * add + 0.1 * dev);
}

float
angular_PID(float angle){
  int i;
  float add,v,v_aux,dev=0.0 ;
  if(angle < -90.0){
    angle = angle + 360.0;
    v_aux= (180.0 - angle)/180.0;
    if(v_aux > 0){
      v = 1 - v_aux;
    }else{
      v = - 1 - v_aux;
    }
  }else{
    v = 1-((180.0 - angle)/180.0);
  }
  integral_angular[cont] = v;
  dev = v - v_prev_angular;
  v_prev_angular = v; 
  for (i=0;i<10;i++){
      add = add + integral_angular[i];
  }
  //ROS_INFO("Add Angular: [%lf]", add);
  //ROS_INFO("V: [%lf]", v);
  return (0.8 * v + 0.1 * dev); //PROBLEMA CON LA ACUMULACIÓN DE ERROR
}

void
forcesCallback(const tfg_jonathan_gines::Forces msg){
  forces[0] = msg.module / 10.0;
  forces[1] = msg.angle;
  ROS_INFO("ForcesTotal Module: [%lf]", forces[0]);
  ROS_INFO("ForcesTotal Angle: [%lf]", forces[1]);
}

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "navigation");		//Inicializa el nodo
  ros::NodeHandle n;
  ros::Publisher velocity_pub;    //Declaramos los publicadores y subcriptores
  ros::Publisher motor_power_pub;
  ros::Subscriber forces_sub;

  float module, angle;

  ros::Rate loop_rate(5);

  geometry_msgs::Twist cmd;
  cmd.linear.x = 0.0; //Velocidad lineal
  cmd.linear.y = 0.0;
  cmd.linear.z = 0.0;
  cmd.angular.x = 0.0;
  cmd.angular.y = 0.0;
  cmd.angular.z = 0.0; //Velocidad angular


  forces_sub 	= n.subscribe("/forces_vector", 1, forcesCallback);
  //velocity_pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1); //ROBOT REAL
  velocity_pub = n.advertise<geometry_msgs::Twist>("/robot/commands/velocity", 1); 
  //motor_power_pub = n.advertise<kobuki_msgs::MotorPower>("motor_power", 1);   //ROBOT REAL
  motor_power_pub = n.advertise<kobuki_msgs::MotorPower>("/robot/commands/motor_power", 1);
  kobuki_msgs::MotorPower power_cmd; //variable de tipo kobuki_msgs::MotorPower
  power_cmd.state = kobuki_msgs::MotorPower::ON; //inicializamos a ON
  motor_power_pub.publish(power_cmd);    // y lo publicamos

  while (ros::ok()){          //bucle principal, aqui entramos y nos mantenemos durante la ej

    if (cont == 10){
        cont = 0;
    }

    if(forces[0]==0 && forces[1]==0){
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      velocity_pub.publish(cmd); 
    }else if(forces[1] < 10.0 && forces[1] > -10.0){
      cmd.linear.x = 0.4;
      cmd.angular.z = 0.0;
      velocity_pub.publish(cmd); 
    }else{
        if(forces[0] > 10){
          module = 10.0;
        }else{
          module = forces[0];
        } 
        angle = forces[1];
        cmd.linear.x = linear_PID(module);
        cmd.angular.z = angular_PID(angle);
        cont++;
        velocity_pub.publish(cmd);    //le pasamos vel y lo publicamos*/
    }
    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;
}