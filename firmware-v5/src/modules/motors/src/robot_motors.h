/**
 * @brief    Swarm Robot Motor Controller Library
 * @details  Controls two DC motors with PWM and direction pins, 
 *           while reading position/speed data from magnetic encoders.
 * @authors  Nuwan Jaliyagoda, Pasan Tennakoon, Dilshani Karunarathna
 * @version  2.0.0
 * @url      N/A
 *
 * ------------------------------------------------------------------------------
 */

#ifndef SW_MOTORS_H
#define SW_MOTORS_H

#include <Arduino.h>

#include "config/definitions.h"
#include "robot_encoder.h"

#define MAX_MOTOR_SPEED 255  // Max PWM duty cycle (full speed)
#define MIN_MOTOR_SPEED 30   // Minimum effective speed before stall

/**
 * @struct EncoderReadings
 * @brief  Holds a pair of encoder counts for left and right motors.
 */
struct EncoderReadings
{
    int left;   // Left motor encoder count
    int right;  // Right motor encoder count
};

/**
 * @class SW_Motors
 * @brief High-level control class for two DC motors with encoder feedback.
 */
class SW_Motors
{
private:
    // Default PWM values for each motor (can be used for calibration)
    uint8_t RIGHT_DEFAULT = 90;
    uint8_t LEFT_DEFAULT  = 90;

    // Motor direction states
    boolean leftMotorDir = 1;      // Current left motor direction (1 = forward, 0 = backward)
    boolean rightMotorDir = 1;     // Current right motor direction
    boolean leftMotorDirOld = 0;   // Previous left motor direction (for change detection)
    boolean rightMotorDirOld = 0;  // Previous right motor direction

    // Current speed values for each motor (absolute PWM duty cycle)
    int16_t rightMotorSpeed = 0;
    int16_t leftMotorSpeed  = 0;

    // Encoders for left and right motors
    RobotEncoder encoderL;
    RobotEncoder encoderR;

public:
    // Constructor / Destructor
    SW_Motors();
    ~SW_Motors();

    // Drift correction offsets (positive or negative) to balance motor speeds
    int8_t leftCorrection  = 0;
    int8_t rightCorrection = 0;

    /**
     * @brief Initializes GPIO pins, PWM channels, and attaches encoders.
     */
    void begin();

    /**
     * @brief Set motor speeds and directions.
     * @param left  Speed for left motor  (-MAX_MOTOR_SPEED .. MAX_MOTOR_SPEED)
     * @param right Speed for right motor (-MAX_MOTOR_SPEED .. MAX_MOTOR_SPEED)
     */
    void write(int16_t left, int16_t right);

    /**
     * @brief Immediately stop both motors.
     */
    void stop();

    /**
     * @brief Stop both motors after a delay.
     * @param delay Time in milliseconds to wait before stopping.
     */
    void stop(int16_t delay);

    /**
     * @brief Temporarily stop PWM output (motor pause without changing direction).
     */
    void pause();

    /**
     * @brief Resume motor speeds from the last pause.
     */
    void resume();

    /**
     * @brief Test sequence to verify motor control and directions.
     */
    void test();

    /**
     * @brief Print current encoder readings to Serial.
     */
    void encoder_print();

    /**
     * @brief Reset both encoders' counts to zero.
     */
    void encoder_reset();

    /**
     * @brief Read current encoder counts into external variables.
     * @param left  Pointer to store left encoder count.
     * @param right Pointer to store right encoder count.
     */
    void encoder_read(int32_t *left, int32_t *right);

    /**
     * @brief Read current encoder counts into a struct.
     * @return EncoderReadings {left, right}
     */
    EncoderReadings encoder_read();
};

#endif
