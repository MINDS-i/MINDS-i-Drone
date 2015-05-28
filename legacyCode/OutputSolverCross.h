
#ifndef OUTPUTSYSTEM_H
#define OUTPUTSYSTEM_H

/*
The following equations were generated with LUquadoutput.jl
They are derived from the LU factorization of a matrix characterizing
	the unit responses to the four inputs, which are
		0: x axis impulse (ccw positive) - rotation around  east/ west axis
		1: y axis impulse (ccw positive) - rotation around north/south axis
		2: z axis impulse (ccw positive) - rotation around the vertical axis
		3: throttle*4 - summed output force
	The output is of the form:
		0: north motor
		1: west  motor
		2: east  motor
		3: south motor

The file is based on a plus flying configuration - The characteristic matrix:
[1 -1 -1 1
 -1 1 -1 1
 1 1 -1 -1
 1 1 1 1]

Simple arithmetical optimizations are left to the gcc
*/
void solveOutputs(int* input, int* output){
	int y[4];
	int out[4];

	y[0] = input[0];
	y[1] = input[1] - y[0]*1.0;
	y[2] = input[2] - y[0]*-1.0 - y[1]*0.0;
	y[3] = input[3] - y[0]*1.0 - y[1]*1.0 - y[2]*-1.0;

	out[3] = y[3]/4.0;
	out[2] = y[2]/-2.0 							         	  - ( 2.0*out[3]/-2.0);
	out[1] = y[1]/2.0 					  - (0.0*out[2]/2.0)  - (-2.0*out[3]/ 2.0);
	out[0] = y[0]/1.0 - (-1.0*out[1]/1.0) - (-1.0*out[2]/1.0) - ( 1.0*out[3]/ 1.0);

	output[0] = out[0];
	output[1] = out[2];
	output[2] = out[1];
	output[3] = out[3];
}

#endif
