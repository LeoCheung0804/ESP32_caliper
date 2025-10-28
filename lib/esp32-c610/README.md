# Project Title: ESP32 Controller

## Description

This project involves an ESP32 program for controlling M2006 M3508 with C610 and C620 ESC using the CAN bus protocol.

## Library Dependencies

- ESP32-TWAI-CAN

## Installation

(Provide detailed instructions on how to install and setup your project here.)

## Usage

The main functionalities of the program are provided by the `C610` class. Here are some of its methods:

- `C610::C610(int can_tx, int can_rx)`: Constructor that initializes the CAN bus transmit and receive pins and creates a mutex semaphore.
- `void C610::setup()`: Sets up the CAN bus communication with specific configurations and starts the timers.
- `void C610::changeMode(int motor_id, MotorMode mode)`: Changes the operational mode of a specific motor.
- `void C610::startRun()`: Starts the tasks for reading and sending data to the motors.
- `void C610::sendTaskRunner(void *parameter)`: Runs in a separate task, waits for a notification, then runs the PID loop and sends a frame of data to the motors.
- `void C610::readTaskRunner(void *parameter)`: Runs in a separate task, waits for a notification, then reads data from the motors.
- `void C610::move(int motor_id, int target)`: Sets the target position, velocity, or current for a specific motor, depending on its mode.
- `void C610::PIDloop()`: Runs the PID control loop for each motor.
- `void C610::sendFrame()`: Sends a frame of data to the motors.
- `void C610::addMotor(int motor_id)`: Adds a new motor to the list of motors.
- `M2006 *C610::findMotorById(int motor_id)`: Finds a motor in the list of motors by its ID.
- `void C610::setPID(int motor_id, PID_config config, float value)`: Sets the PID parameters for a specific motor.
- `bool C610::isMotorAvail(int motor_id)`: Checks if a motor is available.

## Contributing


## License

