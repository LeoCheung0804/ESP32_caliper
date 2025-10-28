#include "C610.h"

TaskHandle_t C610::receiveTask = nullptr;
TaskHandle_t C610::controlTask = nullptr;

// Constructor to initialize CAN bus pins for communication
C610::C610(int can_tx, int can_rx)
{
    this->can_tx = can_tx;
    this->can_rx = can_rx;
    mySemaphore = xSemaphoreCreateMutex();
}

// Setup CAN bus communication with specific configurations
void C610::setup()
{

    controlTimer = timerBegin(CONTROL_TIMER_NUM, 80, true);
    receiveTimer = timerBegin(RECEIVE_TIMER_NUM, 80, true);
    if (controlTimer == NULL || receiveTimer == NULL)
    {
        Serial.println("Failed to start timer");
        while (1)
            ;
    }
    timerAttachInterrupt(controlTimer, &onControlTimer, true);
    timerAlarmWrite(controlTimer, 2000, true);

    timerAttachInterrupt(receiveTimer, &onReceiveTimer, true);
    timerAlarmWrite(receiveTimer, 1000, true);

    ESP32Can.setPins(can_tx, can_rx);
    ESP32Can.setRxQueueSize(5);
    ESP32Can.setTxQueueSize(5);
    ESP32Can.setSpeed(TWAI_SPEED_1000KBPS);

    ESP32Can.begin();
}

// Change the operational mode of a specific motor
void C610::changeMode(int motor_id, MotorMode mode)
{
    M2006 *motor = findMotorById(motor_id);
    if (motor != nullptr)
    {
        SemaphoreLock lock(mySemaphore);
        motor->mode = mode;
        motor->target_pos = motor->pos;
        motor->target_vel = 0;
        motor->target_current = 0;
    }
}

void C610::changePosUnit(int motor_id, MotorPosUnit unit)
{
    M2006 *motor = findMotorById(motor_id);
    if (motor != nullptr)
    {
        SemaphoreLock lock(mySemaphore);
        motor->posUnit = unit;
    }
}
void C610::onControlTimer()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(controlTask, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE)
    {
        portYIELD_FROM_ISR();
    }
}

void C610::onReceiveTimer()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(receiveTask, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE)
    {
        portYIELD_FROM_ISR();
    }
}

void C610::startRun()
{

    xTaskCreatePinnedToCore(readTaskRunner, "readTaskRunner", 10000, this, 1, &receiveTask, 0);
    xTaskCreatePinnedToCore(sendTaskRunner, "sendTaskRunner", 10000, this, 2, &controlTask, 0);
    timerAlarmEnable(controlTimer);
    timerAlarmEnable(receiveTimer);
}

void C610::sendTaskRunner(void *parameter)
{
    C610 *driver = static_cast<C610 *>(parameter);
    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        driver->PIDloop();
        driver->sendFrame();
    }
}

void C610::readTaskRunner(void *parameter)
{
    C610 *driver = static_cast<C610 *>(parameter);
    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        driver->receive();
        driver->receive();
    }
}

void C610::receive()
{

    if (ESP32Can.readFrame(rxFrame, 0))
    {
        int motor_ID = rxFrame.identifier - 0x200;
        M2006 *motorPtr = findMotorById(motor_ID);
        if (motorPtr != nullptr)
        {
            SemaphoreLock lock(mySemaphore);
            M2006 &motorSent = *motorPtr;
            motorSent.avail = true;
            int encoderPos = (rxFrame.data[0] << 8) | rxFrame.data[1];
            int diff = encoderPos - motorSent.lastPos;
            if (diff < -M2006_CNT_PER_ROUND / 2)
            {
                diff += M2006_CNT_PER_ROUND;
            }
            else if (diff > M2006_CNT_PER_ROUND / 2)
            {
                diff -= M2006_CNT_PER_ROUND;
            }
            motorSent.pos += diff;
            motorSent.lastPos = encoderPos;

            int vel = (rxFrame.data[2] << 8) | rxFrame.data[3];
            if (vel > 32767)
            {
                vel -= 65536;
            }
            motorSent.vel = this->expotentialFilter(vel, motorSent.lastVel, 0.1);
            motorSent.lastVel = motorSent.vel;

            int current = (rxFrame.data[4] << 8) | rxFrame.data[5];
            if (current > 32767)
            {
                current -= 65536;
            }
            motorSent.current = current;
        }
    }
}

void C610::move(int motor_id, int target)
{
    SemaphoreLock lock(mySemaphore);
    for (auto &motor : motors)
    {
        if (motor.motor_id == motor_id)
        {
            switch (motor.mode)
            {
            case VELOCITY:
                motor.target_vel = target;
                break;
            case POSITION:
                if (motor.posUnit == DEGREE)
                {
                    float ftarget = (target - motor.intialPosDegree) * M2006_CNT_PER_ROUND * M2006_GEAR_RATIO / 360.0f;
                    motor.target_pos = static_cast<int>(ftarget);
                }
                else
                {
                    motor.target_pos = target;
                }
                break;
            case CURRENT:
                motor.target_current = target;
                break;
            }
        }
    }
}

void C610::PIDloop()
{
    SemaphoreLock lock(mySemaphore);
    for (auto &motor : motors)
    {
        switch (motor.mode)
        {
        case CURRENT:
            motor.target_current = constrain(motor.target_current, -motor.max_current, motor.max_current);
            break;
        case VELOCITY:
            motor.target_vel = constrain(motor.target_vel, -motor.max_velocity, motor.max_velocity);
            motor.target_current = motor.PID_velocity.calculate(motor.target_vel - motor.vel);
            break;
        case POSITION:
            motor.target_vel = motor.PID_position.calculate(motor.target_pos - motor.pos);
            motor.target_vel = constrain(motor.target_vel, -motor.max_velocity, motor.max_velocity);
            motor.target_current = motor.PID_velocity.calculate(motor.target_vel - motor.vel);
            break;
        }
    }
}

void C610::sendFrame()
{
    CanFrame frame;
    uint8_t highByte, lowByte;

    frame.identifier = 0x200;
    frame.data_length_code = 8;

    // create empty frame
    for (int i = 0; i < 8; i++)
    {
        frame.data[i] = 0;
    }

    SemaphoreLock lock(mySemaphore);
    for (auto &motor : motors)
    {
        int motor_ID = motor.motor_id;
        if (motor_ID > 4)
        {
            continue;
        }
        int targetCurrent = motor.target_current;
        encodeToBytes(targetCurrent, highByte, lowByte);
        frame.data[int((motor_ID - 1) * 2)] = highByte;
        frame.data[int((motor_ID - 1) * 2 + 1)] = lowByte;
    }

    ESP32Can.writeFrame(frame);
}

int C610::getMotorPos(int motor_id)
{
    M2006 *motor = findMotorById(motor_id);
    SemaphoreLock lock(mySemaphore);
    if (motor != nullptr)
    {
        return motor->pos;
    }
    else
    {
        return 0;
    }
}

int C610::getMotorTargetPos(int motor_id)
{
    M2006 *motor = findMotorById(motor_id);
    SemaphoreLock lock(mySemaphore);
    if (motor != nullptr)
    {
        return motor->target_pos;
    }
    else
    {
        return 0;
    }
}

int C610::getMotorVel(int motor_id)
{
    M2006 *motor = findMotorById(motor_id);
    SemaphoreLock lock(mySemaphore);
    if (motor != nullptr)
    {
        return motor->vel;
    }
    else
    {
        return 0;
    }
}

int C610::getMotorTargetVel(int motor_id)
{
    M2006 *motor = findMotorById(motor_id);
    SemaphoreLock lock(mySemaphore);
    if (motor != nullptr)
    {
        return motor->target_vel;
    }
    else
    {
        return 0;
    }
}

int C610::getMotorCurrent(int motor_id)
{
    M2006 *motor = findMotorById(motor_id);
    SemaphoreLock lock(mySemaphore);
    if (motor != nullptr)
    {
        return motor->target_current;
    }
    else
    {
        return 0;
    }
}

void C610::encodeToBytes(int value, uint8_t &highByte, uint8_t &lowByte)
{
    if (value < 0)
    {
        value += 1 << 16;
    }

    highByte = (value >> 8) & 0xFF; // Extract the high byte
    lowByte = value & 0xFF;         // Extract the low byte
}

int C610::expotentialFilter(int input, int lastInput, float alpha)
{
    return alpha * input + (1 - alpha) * lastInput;
}

void C610::addMotor(int motor_id)
{
    M2006 newMotor;
    newMotor.motor_id = motor_id;
    motors.push_back(newMotor);
}

M2006 *C610::findMotorById(int motor_id)
{
    SemaphoreLock lock(mySemaphore);
    for (auto &motor : motors)
    {
        if (motor.motor_id == motor_id)
        {
            return &motor; // Return pointer to the found motor
        }
    }
    return nullptr; // Return nullptr if not found
}

void C610::setPID(int motor_id, PID_config config, float value)
{
    M2006 *motor = findMotorById(motor_id);

    if (motor != nullptr)
    {
        SemaphoreLock lock(mySemaphore);
        switch (config)
        {
        case VELOCITY_P:
            motor->PID_velocity.P = value;
            break;
        case VELOCITY_I:
            motor->PID_velocity.I = value;
            break;
        case VELOCITY_D:
            motor->PID_velocity.D = value;
            break;
        case POSITION_P:
            motor->PID_position.P = value;
            break;
        case POSITION_I:
            motor->PID_position.I = value;
            break;
        case POSITION_D:
            motor->PID_position.D = value;
            break;
        }
    }
}

// cannnot detect if motor is disconnected during the program
bool C610::isMotorAvail(int motor_id)
{

    M2006 *motor = findMotorById(motor_id);
    if (motor != nullptr)
    {
        SemaphoreLock lock(mySemaphore);
        return motor->avail;
    }
    else
    {
        return false;
    }
}