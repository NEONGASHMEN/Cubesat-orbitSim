#include<iostream>
#include "navigation.h"
#include "common.h"

using namespace std;


navigation :: navigation(double pqrdot_m[3],double BB_m[3])
{
	double s = 0.3;
	
	if(Bfieldnav_prev[0] + Bfieldnav_prev[1] + Bfieldnav_prev[2] + pqrdotnav_prev[0] + pqrdotnav_prev[1] + pqrdotnav_prev[2] == 0)
	{
   		for(int i = 0;i < 3;i++)
   		{
   			BB_n[i] = BB_m[i];
   			pqrdot_n[i] = pqrdot_m[i];
   		}
   	}
   	
   	else
   	{
    		double BiasEstimate[3] = {0,0,0};
    		double pqrBiasEstimate[3] = {0,0,0};
    		
    		for(int i = 0;i<3;i++)
    		{
    			BB_n[i] = (Bfieldnav_prev[i])*(1-s) + s*(BB_m[i]-BiasEstimate[i]);
    			pqrdot_n[i] = (pqrdotnav_prev[i])*(1-s) + s*(pqrdot_m[i]-pqrBiasEstimate[i]);	
    		}
    	}
    	
    	for(int i =0;i < 3;i++)
    	{
    		Bfieldnav_prev[i] = BB_n[i];
		pqrdotnav_prev[i] = pqrdot_n[i];
	}
}
   	
