/**
 * @brief    Swarm Robot Motor Controller Library
 * @author   Nuwan Jaliyagoda
 * @version  2.0.0
 */

#include "robot_motors.h"
#include "config/pins.h"

// ========================
// Motor PWM Configuration
// ========================
#define LEDC_RESOLUTION_BITS 8     // 8-bit PWM resolution (0–255 duty cycle values)
#define LEDC_BASE_FREQ 5000        // PWM base frequency in Hz
#define LEDC_CHANNEL_A 8           // PWM channel for left motor
#define LEDC_CHANNEL_B 9           // PWM channel for right motor

/*
 * Constructor: Initialize encoders and reset counts
 */
SW_Motors::SW_Motors()
{
    encoderL.setCount(0);                               // Reset left encoder count
    encoderL.attachHalfQuad(PIN_ENC_LA, PIN_ENC_LB);    // Attach left encoder pins (half-quadrature mode)

    encoderR.setCount(0);                               // Reset right encoder count
    encoderR.attachHalfQuad(PIN_ENC_RA, PIN_ENC_RB);    // Attach right encoder pins (half-quadrature mode)
}

/*
 * Destructor: Detach PWM pins from the LEDC channels
 */
SW_Motors::~SW_Motors()
{
    ledcDetachPin(PIN_PWM_R);  // Stop PWM on right motor
    ledcDetachPin(PIN_PWM_L);  // Stop PWM on left motor
}

/**
 * @brief Initialize motor control hardware
 * - Configures direction pins as outputs
 * - Sets up PWM channels for speed control
 * - Attaches PWM to the motor pins
 * - Ensures motors start stopped
 */
void SW_Motors::begin()
{
    // Motor direction control pins
    pinMode(PIN_MOT_R1, OUTPUT);
    pinMode(PIN_MOT_R2, OUTPUT);
    pinMode(PIN_MOT_L1, OUTPUT);
    pinMode(PIN_MOT_L2, OUTPUT);

    // Set up PWM channels
    ledcSetup(LEDC_CHANNEL_A, LEDC_BASE_FREQ, LEDC_RESOLUTION_BITS);
    ledcSetup(LEDC_CHANNEL_B, LEDC_BASE_FREQ, LEDC_RESOLUTION_BITS);

    // Attach PWM outputs to pins
    ledcAttachPin(PIN_PWM_R, LEDC_CHANNEL_A);
    ledcAttachPin(PIN_PWM_L, LEDC_CHANNEL_B);

    // Start with motors off
    ledcWrite(LEDC_CHANNEL_A, 0);
    ledcWrite(LEDC_CHANNEL_B, 0);

    Serial.println(">> Motors\t:enabled,pwm");

    this->write(0, 0);  // Ensure both motors are fully stopped
}

/**
 * @brief Set motor speeds and directions
 * @param leftSpeed  Speed for left motor (-MAX_MOTOR_SPEED to +MAX_MOTOR_SPEED)
 * @param rightSpeed Speed for right motor (-MAX_MOTOR_SPEED to +MAX_MOTOR_SPEED)
 */
void SW_Motors::write(int16_t leftSpeed, int16_t rightSpeed)
{
    // Apply drift correction offsets if moving significantly
    if (leftSpeed > 30)      leftSpeed += leftCorrection;
    else if (leftSpeed < -30) leftSpeed -= leftCorrection;

    if (rightSpeed > 30)      rightSpeed += rightCorrection;
    else if (rightSpeed < -30) rightSpeed -= rightCorrection;

    // Limit values to max allowable range
    leftSpeed  = constrain(leftSpeed,  -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
    rightSpeed = constrain(rightSpeed, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);

    // Determine direction flags (1 = forward, 0 = reverse)
    this->leftMotorDir  = (leftSpeed  >= 0) ? 1 : 0;
    this->rightMotorDir = (rightSpeed >= 0) ? 1 : 0;

    // Update motor direction pins only if direction changes
    if (this->leftMotorDir != this->leftMotorDirOld)
    {
        digitalWrite(PIN_MOT_L1, (this->leftMotorDir) ? LOW : HIGH);
        digitalWrite(PIN_MOT_L2, (this->leftMotorDir) ? HIGH : LOW);
        this->leftMotorDirOld = this->leftMotorDir;
    }
    if (this->rightMotorDir != this->rightMotorDirOld)
    {
        digitalWrite(PIN_MOT_R1, (this->rightMotorDir) ? LOW : HIGH);
        digitalWrite(PIN_MOT_R2, (this->rightMotorDir) ? HIGH : LOW);
        this->rightMotorDirOld = this->rightMotorDir;
    }

    // Store absolute speed values (PWM duty cycle)
    this->rightMotorSpeed = abs(rightSpeed);
    this->leftMotorSpeed  = abs(leftSpeed);

    // Apply PWM duty cycles to motors
    ledcWrite(LEDC_CHANNEL_A, this->leftMotorSpeed);
    ledcWrite(LEDC_CHANNEL_B, this->rightMotorSpeed);
}

/**
 * @brief Immediately stop both motors
 */
void SW_Motors::stop()
{
    this->write(0, 0);
}

/**
 * @brief Stop motors after a specified delay
 * @param d Delay in milliseconds before stopping
 */
void SW_Motors::stop(int16_t d)
{
    delay(d);
    this->write(0, 0);
}

/**
 * @brief Temporarily cut motor power without changing direction
 */
void SW_Motors::pause()
{
    ledcWrite(LEDC_CHANNEL_A, 0);
    ledcWrite(LEDC_CHANNEL_B, 0);
}

/**
 * @brief Resume motors at their previous speeds
 */
void SW_Motors::resume()
{
    ledcWrite(LEDC_CHANNEL_A, this->leftMotorSpeed);
    ledcWrite(LEDC_CHANNEL_B, this->rightMotorSpeed);
}

/**
 * @brief Test routine for motor directions and speed
 * - Spins motors CW, CCW, forward, and backward
 * - Gradually ramps speeds up and down
 */
void SW_Motors::test()
{
    // Counter-clockwise spin
    Serial.println(F("robot: CCW"));
    this->write(-200, 200);
    delay(500);
    this->stop(1500);

    // Clockwise spin
    Serial.println(F("robot: CW"));
    this->write(200, -200);
    delay(500);
    this->stop(1500);

    // Forward acceleration
    Serial.println(F("robot: forward++"));
    for (int i = 0; i < 255; i++)
    {
        this->write(i, i);
        delay(25);
    }
    delay(500);

    // Forward deceleration
    Serial.println(F("robot: forward--"));
    for (int i = 255; i > 0; i--)
    {
        this->write(i, i);
        delay(25);
    }
    this->stop(500);
    delay(2000);

    // Backward acceleration
    Serial.println(F("robot: backward++"));
    for (int i = 0; i < 255; i++)
    {
        this->write(-i, -i);
        delay(25);
    }
    delay(500);

    // Backward deceleration
    Serial.println(F("robot: backward--"));
    for (int i = 255; i > 0; i--)
    {
        this->write(-i, -i);
        delay(25);
    }
    this->stop(500);
    delay(2000);
}

/**
 * @brief Print encoder counts for debugging
 */
void SW_Motors::encoder_print()
{
    int32_t countL = encoderL.getCount();
    int32_t countR = encoderR.getCount();
    Serial.printf("Encoder L: %d, R: %d\n", countL, countR);
    delay(100);
}

/**
 * @brief Reset both encoder counts to zero
 */
void SW_Motors::encoder_reset()
{
    encoderL.clearCount();
    encoderR.clearCount();
}

/**
 * @brief Read encoder counts into external variables
 * @param *left Pointer to store left encoder count
 * @param *right Pointer to store right encoder count
 */
void SW_Motors::encoder_read(int32_t *left, int32_t *right)
{
    *left  = encoderL.getCount();
    *right = encoderR.getCount();
}

/**
 * @brief Read encoder counts and return as a struct
 * @return Struct containing left and right encoder readings
 */
EncoderReadings SW_Motors::encoder_read()
{
    struct EncoderReadings readings;
    readings.left  = encoderL.getCount();
    readings.right = encoderR.getCount();

    return readings;
}
