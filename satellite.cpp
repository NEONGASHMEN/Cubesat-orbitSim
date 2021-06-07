#include<iostream>
#include "satellite.h"
#include "eigen/Eigen/Dense"
#include "tibquat.h"
#include "sensor.h"
#include "navigation.h"
#include "control.h"

using namespace std;
using namespace Eigen;


satellite :: satellite(double state[],double time)
{

	//Orbit params
	MatrixXd orb_radius(3,1);
	orb_radius<<state[0],state[1],state[2];
	double orbit_rad_mod = (6378 + 300)*1000;
	MatrixXd rhat(3,1);
	rhat = orb_radius/orbit_rad_mod;
	double mu = 3.986004418E14;
	
	
	//Satellite velocity
	velocity[0] = state[3];
	velocity[1] = state[4];
	velocity[2] = state[5];
	
	
	//Satellite acceleration
	MatrixXd acc(3,1);
	acc = (-1)*rhat*(mu/(orbit_rad_mod*orbit_rad_mod));
	acceleration[0] = acc(0);
	acceleration[1] = acc(1);
	acceleration[2] = acc(2);
	
	
	//Satellite quarternions
	double pdot,qdot,rdot;
	pdot = state[11];
	qdot = state[12];
	rdot = state[13];
	MatrixXd pqrmat(4,4);
	pqrmat(0,0) = 0;
	pqrmat(0,1) = -pdot; 
	pqrmat(0,2) = -qdot; 
	pqrmat(0,3) = -rdot;
	pqrmat(1,0) = pdot;
	pqrmat(1,1) = 0;
	pqrmat(1,2) = rdot;
	pqrmat(1,3) = -qdot;
	pqrmat(2,0) = qdot;
	pqrmat(2,1) = -rdot; 
	pqrmat(2,2) = 0;
	pqrmat(2,3) = pdot;
	pqrmat(3,0) = rdot;
	pqrmat(3,1) = qdot;
	pqrmat(3,2) = -pdot; 
	pqrmat(3,3) = 0;
	MatrixXd quarternions(4,1);
	quarternions<<state[6],state[7],state[8],state[9];
	MatrixXd Mquart_dot(4,1);
	Mquart_dot = 0.5*(pqrmat*quarternions);
	quart_dot[0] = Mquart_dot(0);
	quart_dot[1] = Mquart_dot(1);
	quart_dot[2] = Mquart_dot(2);
	quart_dot[3] = Mquart_dot(3);
	
	
	//Bfield
	MatrixXd BI(3,1);
	BI<<0,0,0.008;
	tibquat tibquat_obj(state[6],state[7],state[8],state[9]);
	MatrixXd trnsfr_mat(3,3);
	trnsfr_mat<<tibquat_obj.r1,tibquat_obj.r2,tibquat_obj.r3,tibquat_obj.r4,tibquat_obj.r5,tibquat_obj.r6,tibquat_obj.r7,tibquat_obj.r8,tibquat_obj.r9;
	MatrixXd BB(3,1);
	BB = (trnsfr_mat.transpose())*BI;
	
	
	//Sensor
	sensor gyro_magmeter;
	gyro_magmeter.gyro(state[11],state[12],state[13]);
	gyro_magmeter.magnetometer(BB(0),BB(1),BB(1));
		
	//cout<<gyro_magmeter.gyro_measurement[0]<<" "<<gyro_magmeter.gyro_measurement[1]<<" "<<gyro_magmeter.gyro_measurement[2]<<" "<<endl;
	//cout<<gyro_magmeter.magnetometer_measurement[0]<<" "<<gyro_magmeter.magnetometer_measurement[1]<<" "<<gyro_magmeter.magnetometer_measurement[2]<<" "<<endl;
	
	
	//Navigation
	navigation filter(gyro_magmeter.gyro_measurement,gyro_magmeter.magnetometer_measurement);
	//cout<<filter.BB_n[0]<<endl;
	//cout<<filter.BB_n[1]<<endl;
	//cout<<filter.BB_n[2]<<endl;
	
	
	//Control
	double n = 84;
	double A = 0.02;
	Vector3d VmuB(0,0,0);
	Vector3d VBB(BB(0),BB(1),BB(2));
	control torquer(filter.BB_n,filter.pqrdot_n);
	VmuB(0) = (torquer.current[0])*n*A;
	VmuB(1) = (torquer.current[1])*n*A;
	VmuB(2) = (torquer.current[2])*n*A;
	
	debug[0] = torquer.current[0];
	debug[1] = torquer.current[1];
	debug[2] = torquer.current[2];
	
	Vector3d VLMNtorquer(0,0,0);
	VLMNtorquer = VmuB.cross(VBB);
	
	
	//Rotational dynamics
	
	//Inertia
	MatrixXd MI(3,3);
	MI<<0.9,0,0,0,0.9,0,0,0,0.3;
	Vector3d Vpqrdot(state[11],state[12],state[13]);
	MatrixXd Mpqrdot(3,1);
	Mpqrdot<<state[11],state[12],state[13];
	MatrixXd MH(3,1);
	MH = MI*Mpqrdot;
	Vector3d VH(MH(0),MH(1),MH(2));
	Vector3d Vtnet(0,0,0);
	
	Vtnet = (VLMNtorquer - Vpqrdot.cross(VH));
	MatrixXd Mtnet(3,1);
	Mtnet<<Vtnet(0),Vtnet(1),Vtnet(2);
	
	MatrixXd Mpqrddot(3,1);
	Mpqrddot = (MI.inverse())*Mtnet;
	
	pqrddot[0] = Mpqrddot(0);
	pqrddot[1] = Mpqrddot(1);
	pqrddot[2] = Mpqrddot(2);
	
	
	
	////Derivatives////
	//-->velocity
	//-->acceleration
	//-->quart_dot
	//-->pqrddot
	k[0] = velocity[0];
	k[1] = velocity[1];
	k[2] = velocity[2];
	k[3] = acceleration[0];
	k[4] = acceleration[1];
	k[5] = acceleration[2];
	k[6] = quart_dot[0];
	k[7] = quart_dot[1];
	k[8] = quart_dot[2];
	k[9] = quart_dot[3];
	k[10] = pqrddot[0];
	k[11] = pqrddot[1];
	k[12] = pqrddot[2];
	
}
