#include <Arduino.h>
#include <ESP32-TWAI-CAN.hpp>
#include <C610.h>
#include <Wiper.h>
#include <iostream>
#include <string>

// Motor Pin definitions for CAN bus and can ID
#define CAN_TX 17   
#define CAN_RX 16
#define SCREW_MOTOR_ID 1

// Sensor pins and settings
#define SERIAL_TX_PIN 10    
#define SERIAL_RX_PIN 9
#define SENSOR_BAUD 9600

// Global variables to track measurements and positions
float distance = 0.0;      // Current distance reading from sensor
float motorcount = 0.0;    // Target position for motor movement
float currentpos = 0.0;    // Current position of the lead screw

// Initialize wiper control object
Wiper endEffectorWiper(CAN_TX, CAN_RX);

bool isNumericString(const String &str) {
    bool hasDot = false;
    bool hasMinus = false;
    
    for (int i = 0; i < str.length(); i++) {
        if (str.charAt(i) == '.') {
            if (hasDot) return false;    // More than one decimal point
            hasDot = true;
        } 
        else if (str.charAt(i) == '-') {
            if (hasMinus || i != 0) return false;    // Multiple minus signs or minus not at start
            hasMinus = true;
        } 
        else if (!isDigit(str.charAt(i))) {
            return false;    // Not a valid digit
        }
    }
    return true;
}

void reset_sensor(Stream& serial) {
    // Send command with proper terminator (sensor likely expects CR or CR+LF)
    serial.print("C\r");        // or "C\r\n" if sensor needs LF as well
    serial.flush();             // ensure bytes are sent
    delay(100);                 // Wait for reset to complete
}

float measure(Stream& serial) {
    String data = serial.readStringUntil('\r');
    delay(50);
    
    // Validate measurement format
    if (data.length() == 9) {  // Expected length for valid measurement
        char firstChar = data.charAt(0);
        if (firstChar == '+' || firstChar == '-') {
            String numPart = data.substring(1);
            if (isNumericString(numPart)) {
                return data.toFloat();
            } else {
                Serial.println("Error: Invalid number format");
            }
        } else {
            delay(50);
        }
    } else {
        delay(50);
    }
    return -999.0;    // Return error value
}

/**
 * Initial setup routine
 */
void setup() {
    // Initialize main serial communication
    Serial.begin(115200);
    
    // Initialize sensor serial communication with error checking
    Serial1.begin(SENSOR_BAUD, SERIAL_RX_PIN, SERIAL_TX_PIN);
    if (!Serial1) {
        Serial.println("Error initializing sensor serial port");
        while (true);    // Halt if sensor initialization fails
    }
    
    // Initialize sensor
    delay(100);
    reset_sensor(Serial1);
    delay(500);
    
    // Configure and initialize the wiper system
    endEffectorWiper.configureLeadScrewMotor(SCREW_MOTOR_ID, 8);

    endEffectorWiper.connect();
    
    // Perform homing sequence
    Serial.println("Ready, start to home");
    if (!endEffectorWiper.performHoming()) {
        Serial.println("Homing failed");
        while (true);    // Halt if homing fails
    }
    Serial.println("Homing done");
}

/**
 * Main program loop
 */
void loop() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();    // Remove whitespace
        currentpos = endEffectorWiper.getLeadScrewPosition();
        
        // Convert input to uppercase for case-insensitive comparison
        String upperInput = input;
        upperInput.toUpperCase();
        
        // Command processing
        if (upperInput == "S" || upperInput == "STOP") {
            endEffectorWiper.stopOperation();
            Serial.println("stopped");
        } 
        else if (isNumericString(input)) {
            float inputValue = input.toFloat();
            // Add range validation
            if (inputValue >= -140 && inputValue <= 20) {
                motorcount = inputValue;
                endEffectorWiper.moveLeadScrew(motorcount);
            } else {
                Serial.println("Error: Value must be between -140 and 20");
            }
        } 
        else if (upperInput == "C") {
            reset_sensor(Serial1);
            Serial.println("Sensor reset");
        } 
        else if (upperInput == "M") {
            // Read and display sensor measurement
            distance = measure(Serial1);
            if (distance > -999.0) {    // Valid measurement check
                Serial.print("Distance (mm): ");
                Serial.println(distance, 3);    // Display with 3 decimal places
            }else {
                Serial.println("Measurement error");
            }
        }
        else {
            Serial.println("Error: Invalid input");
        }

        Serial.print("Lead screw position (mm): ");
        Serial.println(currentpos);
        // Read and display sensor measurement
        distance = measure(Serial1);
        if (distance > -999.0) {    // Valid measurement check
            Serial.print("Distance (mm): ");
            Serial.println(distance, 3);    // Display with 3 decimal places
        }
    }
    //delay(100);    // Control loop rate
}