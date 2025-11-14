#include "Wiper.h"


Wiper::Wiper(int can_tx, int can_rx)
    : MotorDriver(can_tx, can_rx)
 {
    // Initialize your hardware and variables here
    // For example, setting initial position counters to zero
}

float Wiper::getWiperPosition() {
    // Return current wiper position in degrees 0 - 90 deg
    if (homingDone) {
        return (float(MotorDriver.getMotorPos(wiperID)) - float(wiperZeroCnt)) / (M2006_CNT_PER_ROUND * M2006_GEAR_RATIO / 360.0f);
    }
    else {
        return -1;
    }
}

float Wiper::getLeadScrewPosition() {
    // Return current lead screw position in mm
    if (homingDone) {
        return (MotorDriver.getMotorPos(leadScrewID) - leadScrewZeroCnt) * leadScrewLead / (M2006_CNT_PER_ROUND * M2006_GEAR_RATIO);
    }
    else {
        return -1;
    }
}

void Wiper::moveLeadScrew(float target_mm) {
    // Move lead screw by 'cnt' (implement control logic here)
    if (homingDone) {
        int target = - static_cast<int>((target_mm * M2006_CNT_PER_ROUND * M2006_GEAR_RATIO / leadScrewLead) - leadScrewZeroCnt);
        Serial.print("Move lead screw to ");
        Serial.print(target_mm);
        Serial.print(" mm, target cnt: ");
        Serial.println(target);
        MotorDriver.move(leadScrewID, target);

    }
    else {
        Serial.println("Homing not done, Can't move lead screw");
    }
}

void Wiper::moveWiper(float target_degree) {
    // Move wiper by 'cnt' (implement control logic here)
    if (homingDone) {
        int target = static_cast<int>((target_degree * M2006_CNT_PER_ROUND * M2006_GEAR_RATIO / 360.0f) - wiperZeroCnt);
        MotorDriver.move(wiperID, target);
    }
    else {
        Serial.println("Homing not done, Can't move wiper");
    }
}



bool Wiper::performHoming() {
    // Perform homing operation (find zero position)
    // Return true if successful
    MotorDriver.changeMode(leadScrewID, CURRENT);
    int current_cnt = MotorDriver.getMotorPos(leadScrewID);
    int start_time = millis();
    Serial.print("Start homing lead screw from ");
    Serial.print(current_cnt);
    Serial.print(" start time: ");
    Serial.println(start_time);
    // perform homing within 6s
    MotorDriver.move(leadScrewID, -HOMING_CURRENT);
    delay(1000);
    while (millis() - start_time < HOMING_TIMEOUT) {
        int current_vel = abs (MotorDriver.getMotorVel(leadScrewID));

        if (current_vel < HOMING_TOLERANCE) {
            MotorDriver.changeMode(leadScrewID, POSITION);
            leadScrewZeroCnt = MotorDriver.getMotorPos(leadScrewID);
            Serial.print("Homing done, zero count: ");
            Serial.println(leadScrewZeroCnt);

            homingDone = true;
            return true;
        }

        delay(10);
    }
    return false;
    
}

int Wiper::
getErrorCounts(int motor_id) {
    // Return error counts for a motor
    return MotorDriver.findMotorById(motor_id)->pos - MotorDriver.findMotorById(motor_id)->target_pos;
}

//void Wiper::configureWiperMotor(int id) {
//    // Setup wiper motor driver with 'id'
//    this->wiperID = id;
//    MotorDriver.addMotor(id);
//
//}

void Wiper::configureLeadScrewMotor(int id, int lead) {
    // Setup lead screw motor driver with 'id'
    leadScrewLead = lead;
    this->leadScrewID = id;
    MotorDriver.addMotor(id);
    MotorDriver.findMotorById(id)->max_velocity = 3000;
}

void Wiper::startOperation() {
    // Start operation (e.g., start motors)
}

void Wiper::stopOperation() {
    // Stop operation (e.g., stop motors)
    
}

void Wiper::engageWiper() {
    // Engage mechanism (implement logic here)
}

void Wiper::disengageWiper() {
    // Disengage mechanism (implement logic here)
}

void Wiper::wipe() {
    // Implement the wiping action here
}

bool Wiper::connect() {
    // Connect to the motor driver and initialize the motors
    MotorDriver.setup();
    delay(50);
    MotorDriver.startRun();
    delay(100);

    //while (!MotorDriver.isMotorAvail(wiperID) ) {
    //    Serial.print("Wiper ID ");
    //    Serial.print(wiperID);
    //    Serial.println(" not available, try to connect again");
    //    delay(100);
    //}
    while (!MotorDriver.isMotorAvail(leadScrewID) ) {
        Serial.print("Lead screw ID ");
        Serial.print(leadScrewID);
        Serial.println(" not available, try to connect again");
        delay(100);
    }
    return true;
}

bool Wiper::movewipercurrent(int current) {
    MotorDriver.changeMode(leadScrewID, CURRENT);
    int current_cnt = MotorDriver.getMotorPos(leadScrewID);
    int start_time = millis();
    Serial.print("Start current mode moving lead screw from ");
    Serial.print(current_cnt);
    Serial.print(" start time: ");
    Serial.println(start_time);
    //move motor in negative direction
    MotorDriver.move(leadScrewID, current);
    delay(1000);
    while (millis() - start_time < HOMING_TIMEOUT) {
        int current_vel = abs(MotorDriver.getMotorVel(leadScrewID));
        if (current_vel < HOMING_TOLERANCE) {
            MotorDriver.move(leadScrewID, 0);
            delay(500);
            MotorDriver.move(leadScrewID, MOVING_CURRENT);
            delay(500);
            MotorDriver.move(leadScrewID, -MOVING_CURRENT);
            delay(500);
            MotorDriver.changeMode(leadScrewID, POSITION);
            leadScrewZeroCnt = MotorDriver.getMotorPos(leadScrewID);
            Serial.print("Moving done, zero count: ");
            Serial.println(leadScrewZeroCnt);
            return true;
        }
        delay(1000);
        MotorDriver.changeMode(leadScrewID, POSITION);
        leadScrewZeroCnt = MotorDriver.getMotorPos(leadScrewID);
        Serial.print("position done, zero count: ");
        Serial.println(leadScrewZeroCnt);
    }

    return false;
}