#ifndef ZABERX_H
#define ZABERX_H

#include "Arduino.h"
#define mmResolution 0.000047625
#define umResolution 0.047625


	int readAnalog(int analogPin, int iterations);

	long mm(float mmValue);

	long um(float umValue);

	double interp1(double input);


#endif