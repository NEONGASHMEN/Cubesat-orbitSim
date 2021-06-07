#include<iostream>
#include "control.h"
#include "eigen/Eigen/Dense"
#include<cmath>

using namespace std;
using namespace Eigen;



control :: control(double BB_n[3],double pqrdot_n[3])
{
	double k = 38400;
	Vector3d VBB_n(BB_n[0],BB_n[1],BB_n[2]);
	Vector3d Vpqrdot_n(pqrdot_n[0],pqrdot_n[1],pqrdot_n[2]);
	Vector3d Vcurrent(0,0,0);
	MatrixXd checkval(1,3);
	double n = 84;
	double A = 0.02;
	
	Vcurrent = (k*(Vpqrdot_n.cross(VBB_n)))/(n*A);
	
	
	checkval(0,0) = Vcurrent(0);
	checkval(0,1) = Vcurrent(1);
	checkval(0,2) = Vcurrent(2);
	
	
	checkval = checkval.cwiseAbs();
	
	if(checkval.sum() > 0.04)
	{
		Vcurrent = (Vcurrent/sqrt(pow(Vcurrent(0),2) + pow(Vcurrent(1),2) + pow(Vcurrent(2),2)))*0.04;
	}
	
	for(int i = 0;i < 3;i++)
	{
		current[i] = Vcurrent(i);
	}
}
	
	
