#include "EncoderCounter.h"
#include "Debug.h"
#include "Constants.h"

EncoderCounter::EncoderCounter()
    : pcntUnit_(nullptr)
    , pcntChannelA_(nullptr)
    , pcntChannelB_(nullptr)
    , channelA_(GPIO_NUM_NC)
    , channelB_(GPIO_NUM_NC)
    , offsetPosition_(0)
    , lastPosition_(0)
    , initialized_(false)
{
}

EncoderCounter::~EncoderCounter() {
    if (initialized_) {
        if (pcntUnit_ != nullptr) {
            pcnt_unit_stop(pcntUnit_);
            pcnt_unit_clear_count(pcntUnit_);
            pcnt_del_channel(pcntChannelA_);
            pcnt_del_channel(pcntChannelB_);
            pcnt_del_unit(pcntUnit_);
        }
    }
}

bool EncoderCounter::begin(gpio_num_t channelA, gpio_num_t channelB, int unitId) {
    channelA_ = channelA;
    channelB_ = channelB;

    // Configure GPIO pins with pull-up resistors (for open-collector/open-drain hall sensors)
    gpio_set_direction(channelA_, GPIO_MODE_INPUT);
    gpio_set_pull_mode(channelA_, GPIO_PULLUP_ONLY);

    gpio_set_direction(channelB_, GPIO_MODE_INPUT);
    gpio_set_pull_mode(channelB_, GPIO_PULLUP_ONLY);

    Debug::printlnFeature(DebugFeature::TRANSMISSION, "EncoderCounter: GPIO pins configured with pull-ups");

    // Configure PCNT unit
    pcnt_unit_config_t unit_config = {
        .low_limit = TRANS_ENCODER_MIN_COUNT,
        .high_limit = TRANS_ENCODER_MAX_COUNT,
        .flags = {
            .accum_count = 0  // Don't accumulate on overflow
        }
    };

    esp_err_t err = pcnt_new_unit(&unit_config, &pcntUnit_);
    if (err != ESP_OK) {
        Debug::printfFeature(DebugFeature::TRANSMISSION, "EncoderCounter: Failed to create PCNT unit: %d\n", err);
        return false;
    }

    // Configure glitch filter (1023 APB clock cycles)
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000  // 1µs glitch filter
    };
    err = pcnt_unit_set_glitch_filter(pcntUnit_, &filter_config);
    if (err != ESP_OK) {
        Debug::printfFeature(DebugFeature::TRANSMISSION, "EncoderCounter: Failed to set glitch filter: %d\n", err);
        pcnt_del_unit(pcntUnit_);
        return false;
    }

    // Configure channel A for quadrature decoding
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = channelA_,
        .level_gpio_num = channelB_,
        .flags = {}
    };
    err = pcnt_new_channel(pcntUnit_, &chan_a_config, &pcntChannelA_);
    if (err != ESP_OK) {
        Debug::printfFeature(DebugFeature::TRANSMISSION, "EncoderCounter: Failed to create channel A: %d\n", err);
        pcnt_del_unit(pcntUnit_);
        return false;
    }

    // Set edge and level actions for channel A
    // Standard quadrature: count up when A edges with B low, count down when A edges with B high
    err = pcnt_channel_set_edge_action(pcntChannelA_,
                                       PCNT_CHANNEL_EDGE_ACTION_DECREASE,  // Rising A edge
                                       PCNT_CHANNEL_EDGE_ACTION_INCREASE); // Falling A edge
    if (err != ESP_OK) {
        Debug::printfFeature(DebugFeature::TRANSMISSION, "EncoderCounter: Failed to set channel A edge action: %d\n", err);
        pcnt_del_channel(pcntChannelA_);
        pcnt_del_unit(pcntUnit_);
        return false;
    }

    err = pcnt_channel_set_level_action(pcntChannelA_,
                                        PCNT_CHANNEL_LEVEL_ACTION_INVERSE,  // Invert when B is low
                                        PCNT_CHANNEL_LEVEL_ACTION_KEEP);    // Keep when B is high
    if (err != ESP_OK) {
        Debug::printfFeature(DebugFeature::TRANSMISSION, "EncoderCounter: Failed to set channel A level action: %d\n", err);
        pcnt_del_channel(pcntChannelA_);
        pcnt_del_unit(pcntUnit_);
        return false;
    }

    // Configure channel B for quadrature decoding
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = channelB_,
        .level_gpio_num = channelA_,
        .flags = {}
    };
    err = pcnt_new_channel(pcntUnit_, &chan_b_config, &pcntChannelB_);
    if (err != ESP_OK) {
        Debug::printfFeature(DebugFeature::TRANSMISSION, "EncoderCounter: Failed to create channel B: %d\n", err);
        pcnt_del_channel(pcntChannelA_);
        pcnt_del_unit(pcntUnit_);
        return false;
    }

    // Set edge and level actions for channel B
    // Standard quadrature: count up when B edges with A low, count down when B edges with A high
    err = pcnt_channel_set_edge_action(pcntChannelB_,
                                       PCNT_CHANNEL_EDGE_ACTION_INCREASE,  // Rising B edge
                                       PCNT_CHANNEL_EDGE_ACTION_DECREASE); // Falling B edge
    if (err != ESP_OK) {
        Debug::printfFeature(DebugFeature::TRANSMISSION, "EncoderCounter: Failed to set channel B edge action: %d\n", err);
        pcnt_del_channel(pcntChannelA_);
        pcnt_del_channel(pcntChannelB_);
        pcnt_del_unit(pcntUnit_);
        return false;
    }

    err = pcnt_channel_set_level_action(pcntChannelB_,
                                        PCNT_CHANNEL_LEVEL_ACTION_INVERSE,  // Invert when A is low
                                        PCNT_CHANNEL_LEVEL_ACTION_KEEP);    // Keep when A is high
    if (err != ESP_OK) {
        Debug::printfFeature(DebugFeature::TRANSMISSION, "EncoderCounter: Failed to set channel B level action: %d\n", err);
        pcnt_del_channel(pcntChannelA_);
        pcnt_del_channel(pcntChannelB_);
        pcnt_del_unit(pcntUnit_);
        return false;
    }

    // Enable the PCNT unit
    err = pcnt_unit_enable(pcntUnit_);
    if (err != ESP_OK) {
        Debug::printfFeature(DebugFeature::TRANSMISSION, "EncoderCounter: Failed to enable PCNT unit: %d\n", err);
        pcnt_del_channel(pcntChannelA_);
        pcnt_del_channel(pcntChannelB_);
        pcnt_del_unit(pcntUnit_);
        return false;
    }

    // Clear and start
    pcnt_unit_clear_count(pcntUnit_);
    pcnt_unit_start(pcntUnit_);

    initialized_ = true;

    Debug::printfFeature(DebugFeature::TRANSMISSION, "EncoderCounter: Initialized on unit %d, A=%d, B=%d\n",
                  unitId, channelA_, channelB_);
    return true;
}

int32_t EncoderCounter::getPosition() const {
    if (!initialized_) {
        return 0;
    }

    int count = readHardwareCounter();
    return (int32_t)count + offsetPosition_;
}

void EncoderCounter::setPosition(int32_t position) {
    if (!initialized_) {
        return;
    }

    // Clear hardware counter and set offset to achieve desired position
    pcnt_unit_clear_count(pcntUnit_);
    offsetPosition_ = position;
    lastPosition_ = position;

    Debug::printfFeature(DebugFeature::TRANSMISSION, "EncoderCounter: Position set to %ld\n", position);
}

void EncoderCounter::reset() {
    setPosition(0);
}

int32_t EncoderCounter::getDelta() {
    if (!initialized_) {
        return 0;
    }

    int32_t currentPos = getPosition();
    int32_t delta = currentPos - lastPosition_;
    lastPosition_ = currentPos;
    return delta;
}

int EncoderCounter::readHardwareCounter() const {
    int count = 0;
    pcnt_unit_get_count(pcntUnit_, &count);
    return count;
}

void EncoderCounter::updatePosition() {
    // Reserved for future use (e.g., handling counter overflow)
}

void EncoderCounter::readGPIOStates(int& stateA, int& stateB) const {
    stateA = gpio_get_level(channelA_);
    stateB = gpio_get_level(channelB_);
}
