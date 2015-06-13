/*
 * 
 * 
 * Autor: Jonathan Ginés Clavero (jonathangines@hotmail.com)
 * Fecha: 11/01/2015
 *  
 * Nodo para la navegación por VFF
 * 
 */

//OPENCV PARA DIBUJAR UN CIRCULETE

 /*
}*/  

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/LaserScan.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/core/core.hpp"
#include "std_msgs/String.h"
#include <sstream>
#include <math.h>  
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
  float a = sin (angle*PI/180) * distance;
  float b = cos (angle*PI/180) * distance;
  int aPixels = a * MAXDISTANCE;
  int bPixels = b * MAXDISTANCE;

  cv::circle(image,
    cv::Point(X_ORIGIN + bPixels , Y_ORIGIN - aPixels),5/*tamaño*/,cv::Scalar(255,0,0)
  ,1/*grosor de la linea*/);
}

void
setForces(float matrix[]){
  float sum = 0;
  float index = 0;
  float meanDistance = 0;
  float angle = 0;
  float meanAngle = 0;
  for(int i = 0; i<LENGHT;i++){
    if(matrix[i] < 1){
      sum = sum + matrix[i];
      angle = angle + i;
      index++;
    }else{
      if(sum != 0){
        meanDistance = sum / index;
        meanAngle = (angle / index) / ACCURACY;
        if(meanAngle <= 180 && meanAngle >= 0){
          ROS_INFO("meanDistance: [%f]", meanDistance);
          ROS_INFO("meanAngle: [%f]", meanAngle);
          printObject(meanDistance,meanAngle);
        }
        sum = 0;
        index = 0;
        meanAngle = 0;
        meanDistance = 0;
      }
    }
  }
}

void
buildForceImage(){
  cvNamedWindow( "Force's Image");
  image = cv::Mat::zeros(MAXSIZE, MAXSIZE, CV_8UC3);

  cv::circle(image,
    cv::Point(X_ORIGIN,Y_ORIGIN),15/*tamaño*/,cv::Scalar( 0, 0, 255 )
  ,1/*grosor de la linea*/);
  
  cv::imshow("Force's Image", image);
  waitKey(3);  
}

void 
navigatorCallback(const  sensor_msgs::LaserScan msg){

   printf("------- Hokuyo params -------\n");
   ROS_INFO("Angle_min: [%lf]", msg.angle_min);
   ROS_INFO("Angle_max: [%lf]", msg.angle_max );
   ROS_INFO("Range_min: [%lf]", msg.range_min);
   ROS_INFO("Range_max: [%lf]", msg.range_max);

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
   ROS_INFO("Num Objects : [%d]", numObject);
   numObject = 0;

   cv::imshow("Force's Image", image);
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
