#include "BTS7960Controller.h"
#include "Debug.h"
#include "Constants.h"
#include "driver/ledc.h"

// Static flag to track if motor timer has been configured
static bool motorTimerConfigured = false;

BTS7960Controller::BTS7960Controller()
    : rpwmPin_(GPIO_NUM_NC)
    , lpwmPin_(GPIO_NUM_NC)
    , rpwmChannel_(0)
    , lpwmChannel_(0)
    , currentSpeed_(0)
    , currentRPWM_(0)
    , currentLPWM_(0)
    , initialized_(false)
{
}

BTS7960Controller::~BTS7960Controller() {
    if (initialized_) {
        stop();
    }
}

bool BTS7960Controller::begin(gpio_num_t rpwmPin, gpio_num_t lpwmPin,
                               uint8_t rpwmChannel, uint8_t lpwmChannel) {
    rpwmPin_ = rpwmPin;
    lpwmPin_ = lpwmPin;
    rpwmChannel_ = rpwmChannel;
    lpwmChannel_ = lpwmChannel;

    // Configure LEDC timer for motor PWM (10kHz) - only once for all BTS7960 instances
    if (!motorTimerConfigured) {
        ledc_timer_config_t timer_conf = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .duty_resolution = LEDC_TIMER_8_BIT,
            .timer_num = LEDC_TIMER_1,
            .freq_hz = MOTOR_PWM_FREQ,
            .clk_cfg = LEDC_AUTO_CLK
        };

        esp_err_t err = ledc_timer_config(&timer_conf);
        if (err != ESP_OK) {
            Debug::printfFeature(DebugFeature::BRAKE, "BTS7960: Failed to configure timer: %d\n", err);
            return false;
        }
        motorTimerConfigured = true;
        Debug::printlnFeature(DebugFeature::BRAKE, "BTS7960: Motor timer configured (shared by all instances)");
    }

    // Configure RPWM channel
    ledc_channel_config_t rpwm_conf = {
        .gpio_num = rpwmPin_,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = static_cast<ledc_channel_t>(rpwmChannel_),
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_1,
        .duty = 0,
        .hpoint = 0,
        .flags = {
            .output_invert = 0
        }
    };

    esp_err_t err = ledc_channel_config(&rpwm_conf);
    if (err != ESP_OK) {
        Debug::printfFeature(DebugFeature::BRAKE, "BTS7960: Failed to configure RPWM channel: %d\n", err);
        return false;
    }

    // Configure LPWM channel
    ledc_channel_config_t lpwm_conf = {
        .gpio_num = lpwmPin_,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = static_cast<ledc_channel_t>(lpwmChannel_),
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_1,
        .duty = 0,
        .hpoint = 0,
        .flags = {
            .output_invert = 0
        }
    };

    err = ledc_channel_config(&lpwm_conf);
    if (err != ESP_OK) {
        Debug::printfFeature(DebugFeature::BRAKE, "BTS7960: Failed to configure LPWM channel: %d\n", err);
        return false;
    }

    initialized_ = true;

    // Ensure motor is stopped
    stop();

    Debug::printfFeature(DebugFeature::BRAKE, "BTS7960: Initialized RPWM pin %d ch %d, LPWM pin %d ch %d\n",
                  rpwmPin_, rpwmChannel_, lpwmPin_, lpwmChannel_);
    Debug::printlnFeature(DebugFeature::BRAKE, "BTS7960: Note - Enable pins hardwired to 5V (always active)");
    return true;
}

void BTS7960Controller::setSpeed(int16_t speed) {
    if (!initialized_) {
        Debug::printlnFeature(DebugFeature::BRAKE, "BTS7960: Not initialized");
        return;
    }

    // Clamp speed to valid range
    speed = clamp<int16_t>(speed, (int16_t)MOTOR_MAX_REVERSE, (int16_t)MOTOR_MAX_FORWARD);
    currentSpeed_ = speed;

    if (speed > 0) {
        // Extend actuator
        setRPWM((uint8_t)speed);
        setLPWM(0);
    } else if (speed < 0) {
        // Retract actuator
        setRPWM(0);
        setLPWM((uint8_t)(-speed));
    } else {
        // Stop (coast)
        stop();
    }
}

void BTS7960Controller::stop() {
    if (!initialized_) {
        return;
    }

    // Set both PWM to 0 (coast)
    setRPWM(0);
    setLPWM(0);
    currentSpeed_ = 0;
}

void BTS7960Controller::setRPWM(uint8_t duty) {
    // SAFETY: Prevent simultaneous extend and retract
    // If setting RPWM (extend), automatically clear LPWM (retract)
    if (duty > 0 && currentLPWM_ > 0) {
        Debug::printlnFeature(DebugFeature::BRAKE, "BTS7960 SAFETY: Clearing LPWM (retract) before setting RPWM (extend)");
        ledc_set_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(lpwmChannel_), 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(lpwmChannel_));
        currentLPWM_ = 0;
    }

    ledc_set_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(rpwmChannel_), duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(rpwmChannel_));
    currentRPWM_ = duty;
}

void BTS7960Controller::setLPWM(uint8_t duty) {
    // SAFETY: Prevent simultaneous extend and retract
    // If setting LPWM (retract), automatically clear RPWM (extend)
    if (duty > 0 && currentRPWM_ > 0) {
        Debug::printlnFeature(DebugFeature::BRAKE, "BTS7960 SAFETY: Clearing RPWM (extend) before setting LPWM (retract)");
        ledc_set_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(rpwmChannel_), 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(rpwmChannel_));
        currentRPWM_ = 0;
    }

    ledc_set_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(lpwmChannel_), duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(lpwmChannel_));
    currentLPWM_ = duty;
}
