#include <Arduino.h>
#include <ESP32-TWAI-CAN.hpp>
#include <C610.h>
#include <Wiper.h>
#include <iostream>
#include <string>

#define CAN_TX 17
#define CAN_RX 16

#define WIPE_MOTOR_ID 2
#define SCREW_MOTOR_ID 1

#define SERIAL_RX_PIN 9
#define SERIAL_TX_PIN 10
#define SENSOR_BAUD 9600

float distance = 0;
float motorcount = 0;
float currentpos = 0;

Wiper endEffectorWiper(CAN_TX, CAN_RX);


bool isNumericString(const String &str) {
    bool hasDot = false;
    bool hasMinus = false;
    for (int i = 0; i < str.length(); i++) {
        if (str.charAt(i) == '.') {
            if (hasDot) {
                return false; // More than one dot
            }
            hasDot = true;
        } else if (str.charAt(i) == '-') {
            if (hasMinus || i != 0) {
                return false; // More than one minus or minus not at the start
            }
            hasMinus = true;
        } else if (!isDigit(str.charAt(i))) {
            return false; // Not a digit
        }
    }
    return true;
}

void reset_sensor(Stream& serial) {
    serial.flush();  // Clear output buffer
    serial.write('C');  // Send reset command
    delay(100);
}

float measure(Stream& serial) {
        String data = serial.readStringUntil('\r');
        //Serial.print("Raw data received: ");
        //Serial.println(data);
        delay(50); 
        // Check length and format
        if (data.length() == 9) {  // Verify the string length
            char firstChar = data.charAt(0);
            if (firstChar == '+' || firstChar == '-') {
                // Validate that remaining characters form a valid number
                String numPart = data.substring(1);
                if (isNumericString(numPart)) {
                    float length = data.toFloat();
                    //Serial.print("Valid measurement: ");
                    //Serial.println(length, 3);
                    return length;
                } else {
                    Serial.println("Error: Invalid number format");
                }
            } else {
                //Serial.println("Error: Missing + or - sign");
                delay(50);
            }
        } else {
            //Serial.print("Error: Invalid data length. Received length: ");
            //Serial.println(data.length());
            delay(50);
        }
        //delay(100); // Short delay between readings
    return -999.0; // Return an invalid value instead of 0.0 to indicate error
}


void setup() {
    Serial.begin(115200);
    
    // Add error checking for sensor serial connection
    Serial1.begin(SENSOR_BAUD, SERIAL_RX_PIN, SERIAL_TX_PIN);
    if (!Serial1) {
        Serial.println("Error initializing sensor serial port");
        while (true);
    }
    
    delay(100);  // Give more time for serial to stabilize
    reset_sensor(Serial1);
    delay(500);  // Longer delay after reset
    
    endEffectorWiper.configureLeadScrewMotor(SCREW_MOTOR_ID, 12);
    // endEffectorWiper.configureWiperMotor(WIPE_MOTOR_ID);
    
    endEffectorWiper.connect();
    Serial.println("Ready, start to home");
    if (!endEffectorWiper.performHoming())
    {
        Serial.println("Homing failed");
        while (true)
            ;
    }
    Serial.println("Homing done");
}

void loop() {
    // Read serial console commands
    if (Serial.available() > 0)
    {
        String input = Serial.readStringUntil('\n');
        input.trim(); // Remove any leading/trailing whitespace
        currentpos = endEffectorWiper.getLeadScrewPosition();
        if (input == "stop") {
            endEffectorWiper.stopOperation();
            Serial.println("stopped");

        } else if (isNumericString(input)) {
            motorcount = input.toFloat();
            endEffectorWiper.moveLeadScrew(motorcount);
        } else if (input == "C") {
            reset_sensor(Serial1);
            Serial.println("Sensor reset");
        } else {
            Serial.println("Error: Invalid input");
        }
        Serial.print("Lead screw position (mm): ");
        Serial.println(currentpos);
        Serial.print("\n");
        //delay(100);
    }
    distance = measure(Serial1);
    if (distance > -999.0) {  // Valid measurement
    Serial.print("Distance (mm): ");
    Serial.println(distance, 3);  // Print with 3 decimal places
    }
    delay(100); 
}