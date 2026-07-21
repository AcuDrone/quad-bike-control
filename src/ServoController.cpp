#include "ServoController.h"
#include "Debug.h"
#include "Constants.h"
#include "driver/ledc.h"

ServoController::ServoController()
    : pin_(GPIO_NUM_NC)
    , channel_(0)
    , minUs_(1000)
    , maxUs_(2000)
    , currentUs_(1500)
    , initialized_(false)
{
}

ServoController::~ServoController() {
    if (initialized_) {
        disable();
    }
}

bool ServoController::begin(gpio_num_t pin, uint8_t channel, uint16_t minUs, uint16_t maxUs) {
    return begin(pin, channel, minUs, maxUs, (minUs + maxUs) / 2);
}

bool ServoController::begin(gpio_num_t pin, uint8_t channel, uint16_t minUs, uint16_t maxUs, uint16_t initialUs) {
    pin_ = pin;
    channel_ = channel;
    minUs_ = minUs;
    maxUs_ = maxUs;

    // Configure LEDC timer for servo PWM (50Hz)
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_14_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = SERVO_PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };

    esp_err_t err = ledc_timer_config(&timer_conf);
    if (err != ESP_OK) {
        Debug::printfFeature(DebugFeature::SERVO, "ServoController: Failed to configure timer: %d\n", err);
        return false;
    }

    // Configure LEDC channel
    ledc_channel_config_t channel_conf = {
        .gpio_num = pin_,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = static_cast<ledc_channel_t>(channel_),
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
        .flags = {
            .output_invert = 0
        }
    };

    err = ledc_channel_config(&channel_conf);
    if (err != ESP_OK) {
        Debug::printfFeature(DebugFeature::SERVO, "ServoController: Failed to configure channel: %d\n", err);
        return false;
    }

    initialized_ = true;

    if (initialUs > 0) {
        setMicroseconds(initialUs);
        Debug::printfFeature(DebugFeature::SERVO, "ServoController: Initialized on pin %d, channel %d, us=%d\n", pin_, channel_, initialUs);
    } else {
        currentUs_ = 0;
        Debug::printfFeature(DebugFeature::SERVO, "ServoController: Initialized on pin %d, channel %d (no output)\n", pin_, channel_);
    }

    return true;
}

void ServoController::setMicroseconds(uint16_t us) {
    if (!initialized_) {
        Debug::printlnFeature(DebugFeature::SERVO, "ServoController: Not initialized");
        return;
    }

    // Clamp pulse width to configured range
    us = clamp(us, minUs_, maxUs_);
    currentUs_ = us;

    // Convert to duty cycle and apply
    uint32_t duty = usToDutyCycle(us);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(channel_), duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(channel_));
}

void ServoController::disable() {
    if (!initialized_) {
        return;
    }

    ledc_stop(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(channel_), 0);
    Debug::printfFeature(DebugFeature::SERVO, "ServoController: Disabled channel %d\n", channel_);
}

uint32_t ServoController::usToDutyCycle(uint16_t us) const {
    // 14-bit resolution at 50Hz: duty = (pulse_width_us / 20000us) * 16383
    uint32_t duty = ((uint32_t)us * 16383UL) / 20000UL;
    return duty;
}

