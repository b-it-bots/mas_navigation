/*
 * HomogenousTransform.cpp
 *
 *  Created on: May 21, 2012
 *      Author: matthias
 */

#include "mcr_relative_movements/homogenous_transform.h"
#include <iostream>

using namespace std;
HomogenousTransform ht_from_rpy(float roll, float pitch, float yaw) {
	HomogenousTransform ht;

	ht = Eigen::AngleAxis<float>(roll, Eigen::Vector3f(1,0,0))
		* Eigen::AngleAxis<float>(pitch, Eigen::Vector3f(0,1,0))
		* Eigen::AngleAxis<float>(yaw, Eigen::Vector3f(0,0,1));

	//std::cout << "ht_from_rpy: " << std::endl<< ht.affine() << std::endl;
        
        Normalize(ht);
	return ht;

}

HomogenousTransform ht_from_xyz(float x, float y, float z) {
	HomogenousTransform ht;

	ht = Eigen::Translation<float,3>(x,y,z);

        Normalize(ht);
	//	std::cout << "ht_from_xyz: " << std::endl<< ht.affine() << std::endl;

	return ht;
}

HomogenousTransform ht_from_xyzrpy(float x, float y, float z, float roll, float pitch, float yaw) {
        HomogenousTransform ht;
        ht = ht_from_rpy(roll, pitch, yaw) * ht_from_xyz(x,y,z);
        Normalize(ht);
	return ht;
}

HomogenousTransform ht_from_eul(float z_prior, float y_mid, float z_end){
   
   	HomogenousTransform ht;     
	ht = Eigen::AngleAxis<float>(z_prior, Eigen::Vector3f(0,0,1))
		* Eigen::AngleAxis<float>(y_mid, Eigen::Vector3f(0,1,0))
		* Eigen::AngleAxis<float>(z_end, Eigen::Vector3f(0,0,1));

        Normalize(ht);
        return ht;

}

HomogenousTransform ht_from_dh(float alpha, float d, float a, float theta)
{
   
   	HomogenousTransform ht;     
	ht = Eigen::AngleAxis<float>(alpha, Eigen::Vector3f(1,0,0))
		* Eigen::Translation<float,3>(a,0,0)
		* Eigen::AngleAxis<float>(theta, Eigen::Vector3f(0,0,1))
                * Eigen::Translation<float,3>(0,0,d);
        Normalize(ht);

        return ht;

}
float deg2rad(float degree)
{
	float rad = degree * 3.1416 / 180 ;
	return rad;
}

void Normalize(HomogenousTransform &H)
{
  int i,j;
   for(i= 0; i < 3; i++)
   {
      for(j= 0; j< 4; j++)
      {
       //cout<<"The value of Hij is "<<abs(H(i,j));
        if(abs(H(i,j)) <0.00001)
        {
          H(i,j) = 0; 
        }
      }
   }
}
