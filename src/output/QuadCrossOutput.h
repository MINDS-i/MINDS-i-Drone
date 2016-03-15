#ifndef OUTPUTSYSTEM_H
#define OUTPUTSYSTEM_H

/*
The following equations were generated with LUquadoutput.jl
They are derived from the LU factorization of a matrix characterizing
	the unit responses to the four inputs, which are
		0: pitch ccw+
		1: roll ccw+
		2: z ccw+
		3: summed output force
		equations in the x-y-z = North-East-Down frame
	The output is of the form:
		0: front left motor (ccw)
		1: front right motor (cw)
		2: back right motor (ccw)
		3: back left motor   (cw)

The file is based on a plus flying configuration - The model matrix:
[1 1 -1 -1
 1 -1 -1 1
 1 -1 1 -1
 1 1 1 1]

Simple arithmetical optimizations are left to the gcc
*/
void solveOutputs(const float (&input)[4], float (&out)[4]){
	float y[4];

	y[0] = input[0];
	y[1] = input[1] - y[0]*1.0;
	y[2] = input[2] - y[0]*1.0 - y[1]*1.0;
	y[3] = input[3] - y[0]*1.0 - y[1]*-0.0 - y[2]*1.0;

	out[3] = y[3]/4.0;
	out[2] = y[2]/2.0 							             - (-2.0*out[3]/2.0);
	out[1] = y[1]/-2.0 					 - (0.0*out[2]/-2.0) - (2.0*out[3]/-2.0);
	out[0] = y[0]/1.0 - (1.0*out[1]/1.0) - (-1.0*out[2]/1.0) - (-1.0*out[3]/1.0);
}

#endif
