#ifndef buggyPID_h
#define buggyPID_h

#include "Arduino.h"

class buggyPID
{
    public:
        // Constructor to initialize PID coefficients and output limit
        buggyPID(float P, float I, float D, float outputLimit, float integralLimit = 2000, float outputRampLimit = 50000);

         // PID coefficients and limits
        float P = 0;
        float I = 0;
        float D = 0;
        float outputLimit = 0;
        float integralLimit = 2000;
        float outputRampLimit = 50000;

        // Calculate the PID output based on error input
        float calculate(float error);
        
    protected:
        // Internal state variables for PID computation
        float lastError = 0;
        float output_prev = 0;
        float integral = 0;
        float integral_prev = 0;
        unsigned long timestamp_prev;

};

#endif
