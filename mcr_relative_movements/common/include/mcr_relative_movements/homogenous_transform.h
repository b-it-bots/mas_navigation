/*
 * HomogenousTransform.h
 *
 *  Created on: May 21, 2012
 *      Author: matthias
 *      Modified version: Nirmal Giftsun
 */

#ifndef HOMOGENOUSTRANSFORM_H_
#define HOMOGENOUSTRANSFORM_H_

#include <Eigen/Dense>
#include <math.h>

typedef Eigen::Matrix<float, 1 , 4> DH_Parameter;
typedef Eigen::Matrix<float, Eigen::Dynamic,4>        DH_Parameters;

typedef Eigen::Matrix<float, 1, 8>        JointParameter;

typedef Eigen::Transform<float, 3, Eigen::Affine> 	HomogenousTransform;
typedef Eigen::Matrix<float, 3, 3> 			RotationMatrix;

typedef Eigen::Matrix<float, 1, 6> 			Pose;
typedef HomogenousTransform::TranslationPart		TranslationVector;

#define pi 3.14 
#define HT_R_X 0
#define HT_R_Y 1
#define HT_R_Z 2

#define HT_T_X 0
#define HT_T_Y 1
#define HT_T_Z 2

#define HT_BLOCK_RX 0,0,3,1
#define HT_BLOCK_RY 0,1,3,1
#define HT_BLOCK_RZ 0,2,3,1

float deg2rad(float degree);

HomogenousTransform ht_from_rpy(float roll, float pitch, float yaw);

HomogenousTransform ht_from_xyz(float x, float y, float z);

HomogenousTransform ht_from_xyzrpy(float x, float y, float z, float roll, float pitch, float yaw);

HomogenousTransform ht_from_eul(float z_prior, float y_mid, float z_end);

HomogenousTransform ht_from_dh(float alpha, float d, float a, float theta);

void Normalize(HomogenousTransform &H);

//deg2rad(180),deg2rad(-90),0)


#endif /* HOMOGENOUSTRANSFORM_H_ */
