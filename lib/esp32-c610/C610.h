#ifndef C610_h
#define C610_h

#ifndef ARDUINO
#include <Arduino.h>
#endif

#include <ESP32-TWAI-CAN.hpp>
#include <buggyPID.h>
#include <vector>
#include <cstdint>

#define CONTROL_FREQ 500
#define M2006_CNT_PER_ROUND 8192.f
#define M2006_GEAR_RATIO 22.75f
#define MAX_CURRENT 2000
#define MAX_VELOCITY 8000

#define CONTROL_TIMER_NUM 0
#define RECEIVE_TIMER_NUM 1

enum MotorPosUnit {
    DEGREE,
    CNT
};


enum MotorMode {
    CURRENT,
    VELOCITY,
    POSITION
};

enum PID_config {
    VELOCITY_P,
    VELOCITY_I,
    VELOCITY_D,
    POSITION_P,
    POSITION_I,
    POSITION_D
};

struct M2006 {
    int motor_id;
    int pos = 0;
    int vel = 0;
    int current = 0;

    int intialPosDegree = 0;
    int lastPos = 0;
    int lastVel = 0;

    int target_pos = 0;
    int target_vel = 0;
    int target_current = 0; 

    bool used = false;
    MotorMode mode = CURRENT;
    MotorPosUnit posUnit = CNT;
    // velocity loop that output current
    buggyPID PID_velocity = buggyPID(0.5, 1.5, 0.04, 2000);
    // position loop that output velocity
    buggyPID PID_position = buggyPID(0.12, 0, 0.01, 8000);

    int max_current = MAX_CURRENT;
    int max_velocity = MAX_VELOCITY;

    bool avail = false;
};

class C610 {

public:
    C610(int CAN_TX, int CAN_RX);
    void setup();
    void move(int motor_id, int target);
    void startRun();
    
    void changeMode(int motor_id, MotorMode mode);
    void changePosUnit(int motor_id, MotorPosUnit unit);
    int getMotorPos(int motor_id);
    int getMotorTargetPos(int motor_id);
    int getMotorVel(int motor_id);
    int getMotorTargetVel(int motor_id);
    int getMotorCurrent(int motor_id);
    void addMotor(int motor_id); 
    void setPID(int motor_id, PID_config config, float value);
    bool isMotorAvail(int motor_id);
    int feedbackFreq = 0;

    M2006* findMotorById(int motor_id);
    std::vector<M2006> motors; 

    static void IRAM_ATTR onControlTimer();
    static void IRAM_ATTR onReceiveTimer();
    static C610* instance;
    void setupTimer();


private:
    SemaphoreHandle_t mySemaphore; // Semaphore for thread-safe access
    int baud_rate = 2000000;
    int can_tx;
    int can_rx;
    int motor_id;
    CanFrame rxFrame;
    void sendFrame();
    static void readTaskRunner(void *parameter);
    static void sendTaskRunner(void *parameter);
    void encodeToBytes(int value, uint8_t &highByte, uint8_t &lowByte);

    unsigned long lastSendTime = 0;
    void receive();
    void PIDloop();
    MotorMode mode = CURRENT;
    int expotentialFilter(int input, int lastInput, float alpha);
    int posToRPM(int pos);

    unsigned long lastReceiveTime = 0;

    hw_timer_t *controlTimer = NULL;
    hw_timer_t *receiveTimer = NULL;

    static TaskHandle_t receiveTask;
    static TaskHandle_t controlTask;
};

class SemaphoreLock {
private:
    SemaphoreHandle_t semaphore;
public:
    SemaphoreLock(SemaphoreHandle_t semaphore) : semaphore(semaphore) {
        xSemaphoreTake(semaphore, pdMS_TO_TICKS(1));
    }
    ~SemaphoreLock() {
        xSemaphoreGive(semaphore);
    }
};

#endif