/**
 * @brief    Swarm Robot Motor Controller Library - Encoder Interface
 * @details  Provides an abstraction layer for interfacing magnetic/quadrature encoders
 *           with ESP32's hardware Pulse Counter (PCNT) units. Supports full and half 
 *           quadrature decoding, single-edge counting, and hardware interrupt handling.
 * @author   Nuwan Jaliyagoda
 * @version  2.0.0
 */

#pragma once

#include <Arduino.h>
#include <driver/gpio.h>
#include "driver/pcnt.h" // ESP-IDF Pulse Counter driver

// Depending on target chip, the low-level PCNT register structure would be included
// These are commented out here; only needed for direct register manipulation.
//#if CONFIG_IDF_TARGET_ESP32
//   #include "soc/pcnt_struct.h"
//#elif CONFIG_IDF_TARGET_ESP32S3
//   #include "esp32s3/pcnt_struct.h"
//#else
//   #error "Unsupported target chip"
//#endif

// Max number of encoder instances supported by hardware
#define MAX_ESP32_ENCODERS PCNT_UNIT_MAX

/**
 * @class RobotEncoder
 * @brief Handles encoder signal reading using ESP32's PCNT hardware.
 *
 * Supports:
 *  - Attaching encoders in full quadrature or half quadrature mode
 *  - Counting pulses from magnetic or optical encoders
 *  - Clearing, pausing, and resuming count
 *  - Handling overflow automatically
 */
class RobotEncoder
{
private:
    /**
     * @brief Low-level attachment function for setting up encoder pins and mode
     * @param aPintNumber GPIO pin for channel A
     * @param bPinNumber  GPIO pin for channel B
     * @param fullQuad    true for full quadrature, false for half
     */
    void attach(int aPintNumber, int bPinNumber, boolean fullQuad);

    boolean attached = false; // Flag to check if encoder is configured

    static pcnt_isr_handle_t user_isr_handle; // Handle for Pulse Counter ISR
    bool direction;  // Direction of rotation: true = forward, false = reverse
    bool working;    // Status flag (could indicate if pulses are being received)

public:
    RobotEncoder();
    ~RobotEncoder();

    /**
     * @brief Attach encoder in half quadrature mode
     * @note  Counts only one edge per channel, reduces resolution but lighter processing
     */
    void attachHalfQuad(int aPintNumber, int bPinNumber);

    /**
     * @brief Attach encoder for single-edge counting
     * @note  Only uses channel A, ignores direction unless implemented in software
     */
    void attachSingleEdge(int aPintNumber, int bPinNumber);

    /**
     * @brief Get signed pulse count (direction-aware)
     * @return Current accumulated count
     */
    int32_t getCount();

    /**
     * @brief Get raw hardware count (direct from PCNT unit)
     * @note  May need to be processed to account for overflows
     */
    int32_t getCountRaw();

    /**
     * @brief Reset count to zero
     */
    int32_t clearCount();

    /**
     * @brief Pause pulse counting (hardware-level)
     */
    int32_t pauseCount();

    /**
     * @brief Resume pulse counting
     */
    int32_t resumeCount();

    /**
     * @brief Check if encoder is currently attached
     */
    boolean isAttached() { return attached; }

    /**
     * @brief Manually set encoder count value
     * @param value Desired count
     */
    void setCount(int32_t value);

    // ====== Static members for managing multiple encoders ======
    static RobotEncoder *encoders[MAX_ESP32_ENCODERS]; // List of all encoder objects
    static bool attachedInterrupt; // True if ISR already attached

    // ====== Encoder configuration members ======
    gpio_num_t aPinNumber;   // GPIO pin for encoder channel A
    gpio_num_t bPinNumber;   // GPIO pin for encoder channel B
    pcnt_unit_t unit;        // PCNT unit assigned to this encoder
    bool fullQuad = false;   // Mode: true = full quadrature, false = half
    int countsMode = 2;      // Number of counts per cycle (depends on mode)
    volatile int32_t count = 0; // Accumulated pulse count (updated in ISR)
    pcnt_config_t r_enc_config; // PCNT configuration structure
};
