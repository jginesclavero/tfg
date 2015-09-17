/*
 * 
 * 
 * Autor: Jonathan Ginés Clavero (jonathangines@hotmail.com)
 * Fecha: 11/01/2015
 *  
 * Nodo para la navegación por VFF
 * 
 */

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/LaserScan.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/core/core.hpp"
#include "std_msgs/String.h"
#include <sstream>
#include <math.h>

#include <list>
#include <iostream>
using namespace cv;

#define LENGHT 512
#define ACCURACY 2.8444    // 512 muestras/180 grados que capta el laser.
#define MAXSIZE 600
#define X_ORIGIN 300
#define Y_ORIGIN 450
#define MAXDISTANCE 300
#define PI 3.14159265

float nav_matrix[LENGHT];
bool isObject = false;
int numObject = 0;
Mat image;

struct ObjectParams
{
  float repulsion_module;   //Modulo de la fuerza de repulsión que crean los objetos
  float repulsion_angle;    //Angulo de la fuerza de repulsión
};

std::list<ObjectParams> object_list;  //Lista de objetos que vayamos percibiendo con el laser



void
printMatrix(float matrix[]){
  ROS_INFO("Matriz: ");
  for(int i=0;i<LENGHT;i++){
    printf("%lf ",matrix[i]);
  }
  printf("\n");
}

void
foundObject(){
  if(!isObject){
      numObject ++;
      isObject = true;
  }
}

void
printObject(float distance,float angle){
  angle = angle + 90;  //Corregimos el angulo para que la representación sea coherente
  float a = sin (angle*PI/180) * distance;
  float b = cos (angle*PI/180) * distance;
  int aPixels = a * MAXDISTANCE;
  int bPixels = b * MAXDISTANCE;

  cv::circle(image,
    cv::Point(X_ORIGIN + bPixels , Y_ORIGIN - aPixels),5/*tamaño*/,cv::Scalar(255,0,0)
  ,1 /*grosor de la linea*/);
}

/*void
printVector(std::vector v){
  
}*/

ObjectParams
calcModule(ObjectParams ob1,ObjectParams ob2){
  ObjectParams obResult;

  float theta = std::abs(ob1.repulsion_angle - ob2.repulsion_angle); // Angulo que forman los vectores
  float module = sqrt(pow(ob1.repulsion_module,2) + pow(ob2.repulsion_module,2) 
    + 2*ob1.repulsion_module*ob2.repulsion_module*cos(theta * PI / 180.0));

  if (ob2.repulsion_angle != 0){
    obResult.repulsion_angle = (ob1.repulsion_angle + ob2.repulsion_angle) / 2; //Angulo del vector suma
    obResult.repulsion_module = module;
  }else{
    obResult.repulsion_angle = ob1.repulsion_angle;
    obResult.repulsion_module = ob1.repulsion_module;
  }
  return obResult;

}

std::vector<float>
repulsionVector(std::list<ObjectParams> list){
  std::list<ObjectParams>::const_iterator iterator;
  float sum = 0;
  ObjectParams objectAux;
  objectAux.repulsion_angle = 0;
  objectAux.repulsion_module = 0;
  ObjectParams objectB;
  for(iterator = list.begin();iterator!=list.end();++iterator){
    objectB = *iterator;
    objectAux = calcModule(objectB,objectAux);
  }
  std::vector<float> v(2,0);
  v[0] = objectAux.repulsion_angle; //V[0] PARA EL ANGULO
  v[1] = objectAux.repulsion_module; //V[1] PARA EL MODULO
  return v;
}




void
setForces(float matrix[]){
  float sum = 0;
  float index = 0;
  float meanDistance = 0;
  float angle = 0;
  float meanAngle = 0;
  double forceModule = 0;
  matrix[LENGHT-1] = 1;
  for(int i = 0; i<LENGHT;i++){
    if(matrix[i] < 1){
      sum = sum + matrix[i];
      angle = angle + i;
      index++;
    }else{
      if(sum != 0){
        meanDistance = sum / index;
        forceModule = pow(100 - meanDistance*100,2) / pow(100,2);
        meanAngle = (angle / index) / ACCURACY - 90; //Restamos 90 para que el angulo vaya de 90 a -90

        if(meanAngle <= 90 && meanAngle >= (-90)){

          ObjectParams o;
          o.repulsion_module = forceModule;
          o.repulsion_angle = meanAngle;
          object_list.push_back(o);
          printObject(meanDistance,meanAngle);
        }
        sum = 0;
        angle = 0;
        index = 0;
        meanAngle = 0;
        meanDistance = 0;
      }
    }

  }
  std::vector<float> repulsionV = repulsionVector(object_list);
  printf("---------- Forces ---------\n");
  ROS_INFO("repulsionV[0] (angle): [%f]", repulsionV[0]);
  ROS_INFO("repulsionV[1] (module): [%f]", repulsionV[1]);
  //printVector(v);
  object_list.clear();

}

void
buildForceImage(){
  cvNamedWindow( "Force's Image");
  image = cv::Mat::zeros(MAXSIZE, MAXSIZE, CV_8UC3);
  cv::circle(image,
    cv::Point(X_ORIGIN,Y_ORIGIN),15/*tamaño*/,cv::Scalar( 0, 0, 255 )
  ,1/*grosor de la linea*/);
}

void 
navigatorCallback(const  sensor_msgs::LaserScan msg){
  for (int i = 0; i < msg.ranges.size(); i++){
    if(msg.ranges[i] > 1 || isnan(msg.ranges[i])){
      nav_matrix[i] = 1;
      isObject = false;
    }else{
      nav_matrix[i] = msg.ranges[i];
      foundObject();
    }
  }
   //printMatrix(nav_matrix);
  setForces(nav_matrix);
  printf("------- Hokuyo params -------\n");
  ROS_INFO("Angle_min: [%lf]", msg.angle_min);
  ROS_INFO("Angle_max: [%lf]", msg.angle_max );
  ROS_INFO("Range_min: [%lf]", msg.range_min);
  ROS_INFO("Range_max: [%lf]", msg.range_max);
  ROS_INFO("Num Objects : [%d]", numObject);
  numObject = 0;

  cv::imshow("Force's Image", image);
  buildForceImage();
  waitKey(3);  

 }


int
main(int argc, char** argv)
{

  ros::init(argc, argv, "navigation");		//Inicializa el nodo
  ros::NodeHandle n;
  ros::Publisher nav_pub;		//Declaramos los publicadores y subcriptores
  ros::Subscriber nav_sub;
  ros::Rate loop_rate(5);
  buildForceImage();
  nav_sub 	= n.subscribe("/scan", 1, navigatorCallback);
  ros::spin();
  return 0;
}
