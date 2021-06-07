#include<iostream>
#include "sensor.h"

using namespace std;

void sensor :: gyro(double xval,double yval,double zval)
{
	gyro_measurement[0] = xval + (double (rand()%10)/1000);
	gyro_measurement[1] = yval + (double (rand()%10)/1000);
	gyro_measurement[2] = zval + (double (rand()%10)/1000);
}

void sensor :: magnetometer( double xval,double yval,double zval)
{
	magnetometer_measurement[0] = xval + (double (rand()%10)/10000);
	magnetometer_measurement[1] = xval + (double (rand()%10)/10000);
	magnetometer_measurement[2] = xval + (double (rand()%10)/10000);
}
