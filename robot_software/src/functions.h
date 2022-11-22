#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cstring>
#include <iostream>
float integ(float X0, float dt, float Y)
{
	float S = 0;
	S = X0 + Y * dt;
	return(S);
}

float coriolis(float V, float W)
{
	float a;
	//printf("V:%f W:%f\n", V,W);
	a=V*W*1;// sin(90 deg)=1
	return(a);
}

float midle_value(float arr[], int n)
{
	float S = 0, mid;
	for (int k = 0; k <= n; k++)
	{
		    S += arr[k];
	}
	mid = S / n;
	return (mid);
}
