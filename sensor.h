#ifndef SENSOR_H
#define SENSOR_H

class sensor
{
	public:
	void gyro(double xval,double yval,double zval);
	void magnetometer(double xval,double yval,double zval);
	double gyro_measurement[3];
	double magnetometer_measurement[3];
	
};

#endif
