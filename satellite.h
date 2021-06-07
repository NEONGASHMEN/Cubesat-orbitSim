#ifndef SATELLITE_H
#define SATELLITE_H
#include "eigen/Eigen/Dense"

using namespace std;
using namespace Eigen;

class satellite
{
	public:
	float cs_mass = 2;
	satellite(double state[],double time);
	double acceleration[3];
	double velocity[3];
	double quart_dot[4];
	double pqrddot[3];
	double k[13];
	double debug[3]; 
	
};

#endif
