#ifndef COMPASS_H
#define COMPASS_H

#include <Wire.h>
#include <math.h>

void beginCompass();
void rawCompass(int*x,int*y,int*z);
float getHeading();
float getHeading(int xshift,int xscalar,int yshift,int yscalar);
float getHeadingTiltComp(int xshift, int xscalar,
						 int yshift, int yscalar,
						 double pitch, double roll);

#endif
