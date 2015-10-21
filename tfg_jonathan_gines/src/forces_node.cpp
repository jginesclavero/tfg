/*
 * 
 * 
 * Autor: Jonathan Ginés Clavero (jonathangines@hotmail.com)
 * Fecha: 11/07/2015
 *  
 * Nodo para el calculo del vector de fuerzas del sistema de navegación VFF
 * 
 */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"
#include <tfg_jonathan_gines/Forces.h>
#include <sstream>
#include <math.h>
#include <list>
#include <iostream>
#include "tf/tf.h"

#define LENGHT 512
#define ACCURACY 2.8444    // 512 muestras/180 grados que capta el laser.
#define MAXSIZE 600
#define X_ORIGIN 300
#define Y_ORIGIN 450
#define MAXDISTANCE 300
#define PI 3.14159265

#define X_DESTINATION 5.5
#define Y_DESTINATION 1

 float nav_matrix[LENGHT];
 bool isObject = false;

std::list<std::vector<float> > forces_list;  //Lista de los vectores de todos los puntos que vayamos percibiendo con el laser
											// [0] modulo | [1] angulo
std::list<std::vector<float> > object_forces_list;
std::vector<float> repulsionV(2,0);
std::vector<float> attractionV(2,0);
std::vector<float> destination(2,0);


void
printMatrix(float matrix[]){
	ROS_INFO("Matriz: ");
	for(int i=0;i<LENGHT;i++){
		printf("%lf ",matrix[i]);
	}
	printf("\n");
}

std::vector<float>
addingVectors(std::vector<float> ob1,std::vector<float> ob2){
	std::vector<float> obResult(2,0);

 	float theta = std::abs(ob1[1] - ob2[1]); // Angulo que forman los vectores
  	float cat_op = ob1[0] * sin(theta * PI / 180.0);

  	float module = sqrt(pow(ob1[0],2) + pow(ob2[0],2) 
  		+ 2*ob1[0]*ob2[0]*cos(theta * PI / 180.0));
  	if (ob2[0] != 0){
  		obResult[0] = module;
  		obResult[1] =  asin (cat_op / module) * 180.0 /PI + ob2[1];
  	}else{
  		obResult[0] = ob1[0];
  		obResult[1] = ob1[1];
  	}
  	return obResult;
}

std::vector<float>
repulsionVector(std::list<std::vector<float> > list){
	std::list<std::vector<float> >::const_iterator iterator;
	std::vector<float> objectAux(2,0);
	std::vector<float> objectB(2,0);
	for(iterator = list.begin();iterator!=list.end();++iterator){
		objectB = *iterator;
		objectAux = addingVectors(objectB,objectAux);
	}

	return objectAux;
}

void
setForces(float matrix[]){
	float index = 0;
	for(int i = 0; i<LENGHT;i++){
		if(matrix[i] < 1){
			std::vector<float> v(2,0);
			v[0]= pow(100 - matrix[i]*100,2) / pow(100,2);
			v[1] = (i / ACCURACY) - 90;
			object_forces_list.push_back(v);
			isObject = true;
		}
		if(matrix[i] == 1 && isObject){
			forces_list.push_back(repulsionVector(object_forces_list));
			object_forces_list.clear();
			isObject = false;
		} 
	}
	repulsionV = repulsionVector(forces_list);
	repulsionV[1] = repulsionV[1] - 180;
  //ROS_INFO("repulsionV[1] (angle): [%f]", repulsionV[1]);
	forces_list.clear();
}

void 
laserCallback(const  sensor_msgs::LaserScan msg){
	for (int i = 0; i < msg.ranges.size(); i++){
		if(msg.ranges[i] > 1 || isnan(msg.ranges[i])){
			nav_matrix[i] = 1;
			isObject = false;
		}else{
			nav_matrix[i] = msg.ranges[i];
		}
	}
	nav_matrix[LENGHT-1] = 1;
	setForces(nav_matrix);
  /*
  printf("------- Hokuyo params -------\n");
  ROS_INFO("Angle_min: [%lf]", msg.angle_min);
  ROS_INFO("Angle_max: [%lf]", msg.angle_max );
  ROS_INFO("Range_min: [%lf]", msg.range_min);
  ROS_INFO("Range_max: [%lf]", msg.range_max);
  ROS_INFO("Num Objects : [%d]", numObject);*/
}

void
odometryCallback(const nav_msgs::Odometry odom){
	float moduleAttrac;
  /*ROS_INFO("Position X: [%lf]", msg.pose.pose.position.x);
  ROS_INFO("Position Y: [%lf]", msg.pose.pose.position.y);*/

	std::vector<float> odoV(2,0);
	odoV[0] = odom.pose.pose.position.x;
	odoV[1] = odom.pose.pose.position.y;
  	/*ROS_INFO("dest[0] [%f]", destination[0]);
   	ROS_INFO("dest[1] (angle): [%f]", destination[1]);*/
  	ROS_INFO("odoV[0] X [%f]", odoV[0]);
  	ROS_INFO("odoV[1] Y [%f]", odoV[1]);
	if(destination[0]-odoV[0] < 0.5 && destination[0]-odoV[0] > -0.5 && destination[1]-odoV[1] < 0.5 && destination[1]-odoV[1] > -0.5){
		moduleAttrac = 0;
	}else{
		moduleAttrac= (destination[0]-odoV[0])*(destination[1]-odoV[1]);
	}
	float angleAttrac = atan (moduleAttrac) * 180 / PI;

	tf::Pose pose;
	tf::poseMsgToTF(odom.pose.pose, pose);
	double yaw_angle = tf::getYaw(pose.getRotation()) * 180 /PI;
  	//ROS_INFO("YAW: [%lf]", yaw_angle);

  	attractionV[0] = moduleAttrac * 10;  //PRUEBA

	if(destination[0] < 0 && destination[1] > 0){
		attractionV[1] = angleAttrac + 180.0 - yaw_angle;
  	}else if(destination[0] < 0 && destination[1] < 0){
	  	attractionV[1] = angleAttrac - 180.0 - yaw_angle;    //Normalizamos para que el robot sea el eje
	}else{
		attractionV[1] = angleAttrac - yaw_angle;
	}

  //printf("---------- Attraction Forces ---------\n");
  //ROS_INFO("attractionV[0] (module): [%f]", attractionV[0]);
  //ROS_INFO("attractionV[1] (angle): [%f]", attractionV[1]);
}



int
main(int argc, char** argv)
{
	ros::init(argc, argv, std::string("forces"));		//Inicializa el nodo
	ros::NodeHandle n;
	ros::Publisher forces_pub;		//Declaramos los publicadores y subcriptores
	ros::Subscriber scan_sub;
	ros::Subscriber odom_sub;
	tfg_jonathan_gines::Forces forces_msg;

	ros::Rate loop_rate(5);

	destination[0] = X_DESTINATION;
	destination[1] = Y_DESTINATION;

	scan_sub 	= n.subscribe("/scan", 1, laserCallback);
	odom_sub   = n.subscribe("/odom", 1, odometryCallback);
	forces_pub = n.advertise<tfg_jonathan_gines::Forces>("/forces_vector", 1); 
 
	while (ros::ok()){          //bucle principal, aqui entramos y nos mantenemos durante la ej
  		std::vector<float> forceVector(2,0);
  		if(attractionV[0]!=0){
  			//forceVector = addingVectors(attractionV,repulsionV);
  		}else{
  			ROS_INFO("--------------------HEMOS LLEGADO-----------------------");
  			forceVector[0] = 0;
  			forceVector[1] = 0;
   		}
		/*printf("---------- Result Vector ---------\n");
		ROS_INFO("forceVector[0] (module): [%f]", forceVector[0]);
		ROS_INFO("forceVector[1] (angle): [%f]", forceVector[1]);*/
		forces_msg.module = forceVector[0];
		forces_msg.angle = forceVector[1];
		forces_pub.publish(forces_msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
