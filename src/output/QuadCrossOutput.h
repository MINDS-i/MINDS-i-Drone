#ifndef OUTPUTSYSTEM_H
#define OUTPUTSYSTEM_H

/*
Solves a 4x4 linear system
	the unit responses to the four inputs, which are
		0: pitch ccw+
		1: roll ccw+
		2: z ccw+
		3: total percentage output force
		equations in the x-y-z = North-East-Down frame
	The output is of the form:
		0: front right motor (ccw)
		1: back left motor   (ccw)
		1: front left motor  (cw)
		2: back right motor  (cw)

The file is based on a plus flying configuration - The characteristic matrix:

         #0 #1 #2 #3
model = [ 1 -1  1 -1;  # pitch
         -1  1  1 -1;  # roll
          1  1 -1 -1;  # yaw
          1  1  1  1 ] # throttle

where: model*motors=torques

The solution below is build using the LU factorization because it has been
tested to be more efficient than the matrix inverse
*/
void solveOutputs(const float (&input)[4], float (&output)[4]){
	float y[4];
	float out[4];

	y[0] = input[0];
	y[1] = input[1] - y[0];
	y[2] = input[2] + y[0];
	y[3] = input[3] - y[0] - y[1] - y[2];

	out[3] = y[3]/4.0;
	out[2] = y[2]/2.0                   + out[3];
	out[1] = y[1]/2.0          + out[2];
	out[0] = y[0]/1.0 + out[1] - out[2] + out[3];

	output[0] = out[2];
	output[1] = out[1];
	output[2] = out[0];
	output[3] = out[3];
}

#endif
