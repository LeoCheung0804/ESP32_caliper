#ifndef WIPER_H
#define WIPER_H

#include <Arduino.h>
#include <C610.h>

/**
 * @brief Controls the operation of a wiper and its lead screw for positioning.
 * 
 * The WiperControl class offers methods to get and set the positions of the wiper and lead screw,
 * move them to desired positions, perform a homing procedure, and start/stop the operation.
 */

#define HOMING_TOLERANCE 100 // Tolerance for homing operation in encoder counts.
#define HOMING_TIMEOUT 9000 // Timeout for homing operation in milliseconds.
#define HOMING_CURRENT 600 // Current for homing operation in mA.
#define MOVING_CURRENT 600 // Current for moving operation in mA.


class Wiper {
public:
    Wiper(int can_tx, int can_rx); // Constructor to initialize the wiper and lead screw motors.

    float getWiperPosition(); // Returns the wiper position in degrees.
    float getLeadScrewPosition(); // Returns the lead screw position in millimeters.
    void moveLeadScrew(float target_mm); // Moves the lead screw by a specified number of steps.
    void moveWiper(float target_degree); // Rotates the wiper by a specified number of degrees.
    bool performHoming(); // Executes the homing sequence to find and set the zero positions.
    void configureWiperMotor(int id); // Configures the wiper motor with a given identifier.
    void configureLeadScrewMotor(int id, int lead); // Configures the lead screw motor with a given identifier.
    void startOperation(); // Starts the wiper operation.
    void stopOperation(); // Stops the wiper operation.
    void engageWiper(); // Engages the wiper for operation.
    void disengageWiper(); // Disengages the wiper from operation.
    bool connect(); // Connects to the motor driver and initializes the motors.
    int getErrorCounts(int motor_id); // Returns the error counts for a motor.
    bool movewipercurrent(int current); // Moves the wiper in current mode.

private:
    C610 MotorDriver;   // Motor driver object for controlling the wiper and lead screw.
    uint8_t leadScrewID;
    uint8_t wiperID;
    
    int leadScrewLead;
    int leadScrewZeroCnt = 0;
    int wiperZeroCnt;
    int leadScrewCurrentPosition;
    int wiperCurrentPosition;
    bool wiperEngaged;
    bool homingDone = false;

    void wipe(); // Private method to control the wiping action.
};

#endif // WIPERCONTROL_H
