#include "buggyPID.h"

// Constructor initializes PID controller with coefficients and output limit
buggyPID::buggyPID(float P, float I, float D, float outputLimit, float integralLimit, float outputRampLimit)
    : P(P), I(I), D(D), outputLimit(outputLimit), integralLimit(integralLimit), outputRampLimit(outputRampLimit)
{
    this->integralLimit = outputLimit / 3;
    this->timestamp_prev = micros(); 
}

// Calculate PID output based on the given error
float buggyPID::calculate(float error)
{
    unsigned long timestamp = micros();
    float dt = (timestamp - this->timestamp_prev) / 1000000.0;
    
    // Calculate proportional term
    float proportional = error * this->P;

    // Calculate integral term with anti-windup via integral limit check
    float integral = this->integral_prev + 0.5 * this->I * (error + this->lastError) * dt;
    if (this->integralLimit != 0)
    {
        integral = constrain(integral, -this->integralLimit, this->integralLimit);
    }

     // Calculate derivative term
    float derivative = this->D * (error - this->lastError) / dt;

    // Calculate total PID output
    float output = proportional + integral + derivative;

    // Limit the rate of change of the PID output
    float outputChange = (output - this->output_prev) / dt;
    if (outputChange > this->outputRampLimit){
        output = this->output_prev + this->outputRampLimit * dt;
    }
    else if (outputChange < -this->outputRampLimit){
        output = this->output_prev - this->outputRampLimit * dt;
    }
    
    this->output_prev = output;
    this->integral_prev = integral;
    this->timestamp_prev = timestamp;
    this->lastError = error;
    
    // Limit the PID output
    output = constrain(output, -this->outputLimit, this->outputLimit);
    return output;
}