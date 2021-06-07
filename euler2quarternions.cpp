#include<iostream>
//#include "eigen/Eigen/Dense"
#include<cmath>
#include "euler2quarternions.h"

using namespace std;
//using namespace Eigen;

	
euler2quarternions :: euler2quarternions(double phi,double theta,double psi)
	{
		
		q0 = cos(phi/2)*cos(theta/2)*cos(psi/2) + sin(phi/2)*sin(theta/2)*sin(psi/2);
		q1 = sin(phi/2)*cos(theta/2)*cos(psi/2) - cos(phi/2)*sin(theta/2)*sin(psi/2);
		q2= cos(phi/2)*sin(theta/2)*cos(psi/2) + sin(phi/2)*cos(theta/2)*sin(psi/2);
		q3 = cos(phi/2)*cos(theta/2)*sin(psi/2) - sin(phi/2)*sin(theta/2)*cos(psi/2);
		
	}



	

	
