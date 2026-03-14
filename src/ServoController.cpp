#include "ServoController.h"
#include "Debug.h"
#include "Constants.h"
#include "driver/ledc.h"

ServoController::ServoController()
    : pin_(GPIO_NUM_NC)
    , channel_(0)
    , minUs_(1000)
    , maxUs_(2000)
    , currentAngle_(90.0f)
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
    pin_ = pin;
    channel_ = channel;
    minUs_ = minUs;
    maxUs_ = maxUs;

    // Configure LEDC timer for servo PWM (50Hz)
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_16_BIT,
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

    // Set to center position
    setAngle(90.0f);

    Debug::printfFeature(DebugFeature::SERVO, "ServoController: Initialized on pin %d, channel %d\n", pin_, channel_);
    return true;
}

void ServoController::setAngle(float degrees) {
    if (!initialized_) {
        Debug::printlnFeature(DebugFeature::SERVO, "ServoController: Not initialized");
        return;
    }

    // Clamp angle to 0-180 range
    degrees = clamp(degrees, 0.0f, 180.0f);
    currentAngle_ = degrees;

    // Convert angle to pulse width
    uint16_t us = angleToPulseWidth(degrees);
    setMicroseconds(us);
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

uint16_t ServoController::angleToPulseWidth(float degrees) const {
    // Map 0-180 degrees to minUs-maxUs microseconds
    float ratio = degrees / 180.0f;
    uint16_t us = minUs_ + (uint16_t)((maxUs_ - minUs_) * ratio);
    return us;
}

uint32_t ServoController::usToDutyCycle(uint16_t us) const {
    // Calculate duty cycle for 16-bit resolution at 50Hz
    // Period = 1/50Hz = 20ms = 20000us
    // Duty cycle = (pulse_width_us / 20000us) * 65535
    uint32_t duty = ((uint32_t)us * 65535UL) / 20000UL;
    return duty;
}
