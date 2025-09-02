/**
 * @brief    Swarm Robot Motor Controller Library - Encoder Implementation
 * @version  2.0.0
 *
 * This file implements the RobotEncoder class for reading rotary/magnetic encoders
 * using the ESP32's PCNT (Pulse Counter) hardware peripheral.
 * It supports full/half quadrature decoding and handles counter overflow
 * through an interrupt service routine (ISR).
 */

#include "driver/pcnt.h"
#include "robot_encoder.h"
#include "config/pins.h"
#include "soc/pcnt_struct.h" // Access PCNT registers for ISR status and clearing

// ===== Static Class Variables =====
RobotEncoder *RobotEncoder::encoders[MAX_ESP32_ENCODERS] = {NULL, NULL, NULL, NULL}; 
bool RobotEncoder::attachedInterrupt = false;  // Tracks if ISR is already registered
pcnt_isr_handle_t RobotEncoder::user_isr_handle = NULL; // ISR handle for PCNT

// ===== Constructor =====
RobotEncoder::RobotEncoder()
{
    attached = false;      // Encoder is not yet configured
    aPinNumber = (gpio_num_t)0;
    bPinNumber = (gpio_num_t)0;
    working = false;
    direction = false;
    unit = (pcnt_unit_t)0;
}

// Destructor (not used here but defined for completeness)
RobotEncoder::~RobotEncoder() {}

/**
 * @brief ISR for handling PCNT overflow events (high/low limit reached).
 * @note  Runs in IRAM (must be short and fast). Fully compatible with ESP32-S3.
 *
 * The PCNT unit only counts up to ±32767 (16-bit). When overflow occurs,
 * this ISR adjusts the software-maintained `count` variable accordingly.
 */
static void IRAM_ATTR pcnt_example_intr_handler(void *arg)
{
    RobotEncoder *ptr;
    uint32_t intr_status = PCNT.int_st.val; // Read interrupt status for all PCNT units

    for (int i = 0; i < PCNT_UNIT_MAX; i++)
    {
        if (intr_status & (1 << i)) // If this unit triggered the interrupt
        {
            ptr = RobotEncoder::encoders[i]; // Get encoder object
            uint32_t evt_status = 0;
            pcnt_get_event_status((pcnt_unit_t)i, &evt_status);

            int status = 0;

            // Check if high limit was reached (positive overflow)
            if (evt_status & PCNT_EVT_H_LIM)
            {
                status = ptr->r_enc_config.counter_h_lim;
            }
            // Check if low limit was reached (negative overflow)
            else if (evt_status & PCNT_EVT_L_LIM)
            {
                status = ptr->r_enc_config.counter_l_lim;
            }

            PCNT.int_clr.val = (1 << i); // Clear the interrupt flag
            ptr->count += status;        // Adjust software counter
        }
    }
}

/**
 * @brief Core function for attaching an encoder to a PCNT unit.
 * @param a   GPIO number for channel A
 * @param b   GPIO number for channel B
 * @param fq  true = full quadrature mode, false = single-edge mode
 *
 * Configures GPIOs, sets PCNT counting modes, enables noise filtering,
 * and registers ISR if not already done.
 */
void RobotEncoder::attach(int a, int b, boolean fq)
{
    // Avoid attaching twice to the same object
    if (attached)
    {
        Serial.println("Encoder already attached!");
        return;
    }

    // Assign next available PCNT unit
    int index = 0;
    for (; index < MAX_ESP32_ENCODERS; index++)
    {
        if (RobotEncoder::encoders[index] == NULL)
        {
            encoders[index] = this;
            break;
        }
    }
    if (index == MAX_ESP32_ENCODERS)
    {
        Serial.println("Too many encoders, attach failed!");
        return;
    }

    // Store configuration
    fullQuad = fq;
    unit = (pcnt_unit_t)index;
    this->aPinNumber = (gpio_num_t)a;
    this->bPinNumber = (gpio_num_t)b;

    // ===== GPIO Configuration =====
    gpio_pad_select_gpio(aPinNumber);
    gpio_pad_select_gpio(bPinNumber);
    gpio_set_direction(aPinNumber, GPIO_MODE_INPUT);
    gpio_set_direction(bPinNumber, GPIO_MODE_INPUT);
    gpio_pulldown_en(aPinNumber);
    gpio_pulldown_en(bPinNumber);

    // ===== PCNT Configuration =====
    r_enc_config.pulse_gpio_num = aPinNumber;    // Pulse input (channel A)
    r_enc_config.ctrl_gpio_num = bPinNumber;     // Control input (channel B)
    r_enc_config.unit = unit;
    r_enc_config.channel = PCNT_CHANNEL_0;

    // Set count modes depending on full/half quadrature
    r_enc_config.pos_mode = fullQuad ? PCNT_COUNT_DEC : PCNT_COUNT_DIS; // On rising edge
    r_enc_config.neg_mode = PCNT_COUNT_INC;                             // On falling edge
    r_enc_config.lctrl_mode = PCNT_MODE_KEEP;     // Low control: keep direction
    r_enc_config.hctrl_mode = PCNT_MODE_REVERSE;  // High control: reverse direction

    // Set counter limits for overflow detection
    r_enc_config.counter_h_lim = INT16_MAX;
    r_enc_config.counter_l_lim = INT16_MIN;

    pcnt_unit_config(&r_enc_config); // Apply configuration

    // Enable noise filter to reject spurious pulses
    pcnt_set_filter_value(unit, 250); 
    pcnt_filter_enable(unit);

    // Enable events for overflow detection
    pcnt_event_enable(unit, PCNT_EVT_H_LIM);
    pcnt_event_enable(unit, PCNT_EVT_L_LIM);

    // Reset and pause counter until ready
    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);

    // Register ISR if not already done
    if (!RobotEncoder::attachedInterrupt)
    {
        RobotEncoder::attachedInterrupt = true;
        esp_err_t er = pcnt_isr_register(pcnt_example_intr_handler, NULL, 0, &RobotEncoder::user_isr_handle);
        if (er != ESP_OK)
        {
            Serial.println("ISR registration failed");
        }
    }

    // Enable interrupts for this PCNT unit
    pcnt_intr_enable(unit);

    // Start counting
    pcnt_counter_resume(unit);

    attached = true; // Mark encoder as active
}

/**
 * @brief Attach encoder in half quadrature mode.
 * @note  Counts on one channel with direction from the other.
 */
void RobotEncoder::attachHalfQuad(int aPintNumber, int bPinNumber)
{
    attach(aPintNumber, bPinNumber, true);
}

/**
 * @brief Attach encoder in single-edge mode.
 * @note  Counts only on one edge, ignores one channel.
 */
void RobotEncoder::attachSingleEdge(int aPintNumber, int bPinNumber)
{
    attach(aPintNumber, bPinNumber, false);
}

/**
 * @brief Set logical encoder count value.
 * @param value New count value
 * @note  Adjusts software offset so hardware count maps to desired value.
 */
void RobotEncoder::setCount(int32_t value)
{
    count = value - getCountRaw();
}

/**
 * @brief Get raw hardware count from PCNT unit.
 */
int32_t RobotEncoder::getCountRaw()
{
    int16_t c;
    pcnt_get_counter_value(unit, &c);
    return c;
}

/**
 * @brief Get total logical count including software-adjusted overflow tracking.
 */
int32_t RobotEncoder::getCount()
{
    return getCountRaw() + count;
}

/**
 * @brief Clear count to zero.
 */
int32_t RobotEncoder::clearCount()
{
    count = 0;
    return pcnt_counter_clear(unit);
}

/**
 * @brief Pause counting without resetting the count.
 */
int32_t RobotEncoder::pauseCount()
{
    return pcnt_counter_pause(unit);
}

/**
 * @brief Resume counting after pause.
 */
int32_t RobotEncoder::resumeCount()
{
    return pcnt_counter_resume(unit);
}
