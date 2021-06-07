//_mod shall be post fixed for moduli of vectors

#include<iostream>
#include "eigen/Eigen/Dense"
#include<cmath>
#include "euler2quarternions.h"
#include "satellite.h"


using namespace std;
using namespace Eigen;


//Earth params
double mu = 3.986004418E14;
double radius_of_earth = 6738*1000;



//orbit params
double orbit_rad_mod = (6378 + 300)*1000;
const double pi = 3.14159265358979323846;
double inclination = (98*pi)/180;
double velocity_mod = sqrt(mu/orbit_rad_mod);
double x = orbit_rad_mod;
double y = 0;
double z = 0; 
double xdot = 0;
double ydot = velocity_mod*cos(inclination);
double zdot = velocity_mod*sin(inclination);
double period = 2*pi*sqrt(pow(orbit_rad_mod,3)/mu);
MatrixXd velocity(3,1);


//satellite params
double p = 0;
double q = 0;
double r = 0;
double pdot = 0.175;
double qdot = 0.175*0;
double rdot = 0.175*0;
MatrixXd quarternions(4,1);

//state
MatrixXd state(1,13);

int main()
{

	//velocity matrix
	velocity(0,0) = xdot;
	velocity(1,0) = ydot;
	velocity(2,0) = zdot;
	//Matrix <float, Dynamic, Dynamic> matrixA;
	//MatrixXf matrixA(10,10);
	//matrixA(0,0) = 5;
	//matrixA(1,0) = 6;
	//matrixA(2,0) = 7;
	//cout<<matrixA <<endl;
	euler2quarternions e2q(p,q,r);
	quarternions<<e2q.q0,e2q.q1,e2q.q2,e2q.q3;
	
	
	//state assignment
	state<<x,y,z,xdot,ydot,zdot,quarternions(0,0),quarternions(1,0),quarternions(2,0),quarternions(3,0),pdot,qdot,rdot;
	//cout<<state<<endl;
	
	double *state_in_c = state.data();
	
	//Output matrices initialisation
	double timestep = 1;
	double no_of_orbs = 15;
	double total_time = period*no_of_orbs;
	int no_of_opt = floor(total_time/timestep)+1;
	MatrixXd stateout(no_of_opt,13);
	
	
	double k1[13];
	double k2[13];
	double k3[13];
	double k4[13];
	double main_k[13];
	
	//RK4
	for(int i = 0;i < total_time;i = i + timestep)
	{
		for(int j = 0;j < 13;j++)
		{
			stateout(i,j) = state_in_c[j];
		}
		
		
		double state_in_c_k1[13];
		double state_in_c_k2[13];
		double state_in_c_k3[13];
		
		satellite cubesat_k1(state_in_c,i);
		//cout<<cubesat_k1.debug[0]<<"  "<<cubesat_k1.debug[1]<<"  "<<cubesat_k1.debug[2]<<"  "<<endl;
		for(int j = 0;j < 13;j++)
		{
			k1[j] = cubesat_k1.k[j];
		}
		for(int j = 0;j < 13;j++)
		{
			state_in_c_k1[j] = state_in_c[j] + k1[j]*(timestep/2);
		}
		//-------------------------------------------------------------------//
		
		satellite cubesat_k2(state_in_c_k1,i + timestep/2);
		for(int j = 0;j < 13;j++)
		{
			k2[j] = cubesat_k2.k[j];
		}
		for(int j = 0;j < 13;j++)
		{
			state_in_c_k2[j] = state_in_c[j] + k2[j]*(timestep/2);
		}
		//-------------------------------------------------------------------//
		
		satellite cubesat_k3(state_in_c_k2,i + timestep/2);
		for(int j = 0;j < 13;j++)
		{
			k3[j] = cubesat_k3.k[j];
		}
		for(int j = 0;j < 13;j++)
		{
			state_in_c_k3[j] = state_in_c[j] + k3[j]*(timestep);
		}
		//-------------------------------------------------------------------//
		
		satellite cubesat_k4(state_in_c_k3,i + timestep);
		for(int j = 0;j < 13;j++)
		{
			k4[j] = cubesat_k4.k[j];
		}
		//-------------------------------------------------------------------//
		
		for(int j = 0;j < 13;j++)
		{
			main_k[j] = (k1[j] + 2*k2[j] + 2*k3[j] + k4[j])/6;
		}
		//-------------------------------------------------------------------//
		
		for(int j = 0;j < 13;j++)
		{
			state_in_c[j] = state_in_c[j] + main_k[j]*timestep;
		}
		
	}
	
	for(int i = 0;i < no_of_opt;i++)
	{
		stateout(i,0) = stateout(i,0)/1000;
		stateout(i,1) = stateout(i,1)/1000;
		stateout(i,2) = stateout(i,2)/1000;
		stateout(i,3) = stateout(i,3)/1000;
		stateout(i,4) = stateout(i,4)/1000;
		stateout(i,5) = stateout(i,5)/1000;
	}
	cout<<stateout<<endl;
	
	
	return 0;
}
	
