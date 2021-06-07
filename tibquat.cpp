#include<iostream>
#include "tibquat.h"
#include<cmath>

using namespace std;

tibquat :: tibquat(double q0,double q1,double q2,double q3)
{
	r1 = (pow(q0,2)+pow(q1,2)-pow(q2,2)-pow(q3,2));
	r2 = 2*(q1*q2-q0*q3);
	r3 = 2*(q0*q2+q1*q3);
        r4 = 2*(q1*q2+q0*q3);
        r5 = (pow(q0,2)-pow(q1,2)+pow(q2,2)-pow(q3,2)); 
        r6 = 2*(q2*q3-q0*q1);
        r7 = 2*(q1*q3-q0*q2); 
        r8 = 2*(q0*q1+q2*q3); 
        r9 = (pow(q0,2)-pow(q1,2)-pow(q2,2)+pow(q3,2));
}
