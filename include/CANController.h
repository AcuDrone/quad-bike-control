#ifndef CAN_CONTROLLER_H
#define CAN_CONTROLLER_H

#include <Arduino.h>
#include <mcp_can.h>
#include "Constants.h"

/**
 * @brief CAN bus controller for reading vehicle OBD-II data via MCP2515
 *
 * Interfaces with MCP2515 SPI CAN controller to read standard OBD-II diagnostic data
 * from vehicle ECU (engine RPM, speed, coolant temperature, etc.).
 * Uses a non-blocking state machine to avoid stalling the main loop.
 */
class CANController {
public:
    /**
     * @brief Vehicle data structure from CAN bus
     */
    struct VehicleData {
        uint16_t engineRPM;         // RPM (0-16383)
        uint8_t vehicleSpeed;       // km/h (0-255)
        int8_t coolantTemp;         // °C (-40 to +215)
        int8_t oilTemp;             // °C (-40 to +215)
        uint8_t throttlePosition;   // % (0-100)
        uint8_t fuelLevel;          // % (0-100)
        uint32_t lastUpdateTime;    // millis() timestamp of last successful update
        bool dataValid;             // true if CAN communication is healthy
    };

    CANController();
    ~CANController();

    /**
     * @brief Initialize CAN controller
     *
     * @param csPin Chip Select pin for MCP2515
     * @param sckPin SPI Clock pin
     * @param mosiPin SPI MOSI pin
     * @param misoPin SPI MISO pin
     * @return true if initialization successful
     */
    bool begin(gpio_num_t csPin = PIN_CAN_CS,
               gpio_num_t sckPin = PIN_CAN_SCK,
               gpio_num_t mosiPin = PIN_CAN_MOSI,
               gpio_num_t misoPin = PIN_CAN_MISO);

    /**
     * @brief Update CAN controller - call every loop iteration
     * Non-blocking state machine: sends one request or checks one response per call
     */
    void update();

    /**
     * @brief Get current vehicle data
     * @return VehicleData structure with latest values
     */
    VehicleData getVehicleData() const { return vehicleData_; }

    /**
     * @brief Check if CAN controller is connected and responding
     * @return true if MCP2515 is responding
     */
    bool isConnected() const { return initialized_; }

    /**
     * @brief Get human-readable status string
     * @return Status string (e.g., "Connected - 10 Hz", "Disconnected")
     */
    String getStatusString() const;

private:
    // State machine states
    enum class OBDState : uint8_t {
        IDLE,               // Ready to send next PID request
        WAITING_RESPONSE    // Request sent, polling for response
    };

    // PID scheduling entry
    struct PIDEntry {
        uint8_t pid;
        uint32_t interval;       // ms between polls
        uint32_t nextPollTime;   // millis() when next poll is due
        uint8_t retryCount;
    };

    static constexpr uint8_t PID_COUNT = 6;

    // OBD-II PIDs (Mode 01 - Current Data)
    static constexpr uint8_t PID_ENGINE_RPM = 0x0C;
    static constexpr uint8_t PID_VEHICLE_SPEED = 0x0D;
    static constexpr uint8_t PID_COOLANT_TEMP = 0x05;
    static constexpr uint8_t PID_OIL_TEMP = 0x5C;
    static constexpr uint8_t PID_THROTTLE_POS = 0x11;
    static constexpr uint8_t PID_FUEL_LEVEL = 0x2F;

    MCP_CAN* mcp_can_;              // MCP2515 CAN controller object
    VehicleData vehicleData_;       // Current vehicle data
    bool initialized_;              // Initialization status

    // State machine
    OBDState state_;
    uint8_t activePIDIndex_;        // Index into pidTable_ of current request
    uint32_t requestSentTime_;      // millis() when current request was sent

    // PID scheduling table
    PIDEntry pidTable_[PID_COUNT];

    // Response buffer
    uint8_t responseData_[5];
    uint8_t responseLen_;

    bool sendOBDRequest(uint8_t pid);
    bool tryReceiveResponse();
    int8_t selectNextPID();
    void parseAndStore(uint8_t index, const uint8_t* data, uint8_t len);
    bool isDataStale() const;
};

#endif // CAN_CONTROLLER_H
