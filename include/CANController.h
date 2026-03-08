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
     * Polls OBD-II data based on configured intervals
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
    MCP_CAN* mcp_can_;              // MCP2515 CAN controller object
    VehicleData vehicleData_;       // Current vehicle data
    bool initialized_;              // Initialization status
    uint32_t lastRPMPoll_;          // Last RPM/speed poll time
    uint32_t lastTempPoll_;         // Last temperature poll time
    uint8_t retryCount_;            // Current retry count for failed requests

    // OBD-II PIDs (Mode 01 - Current Data)
    static constexpr uint8_t PID_ENGINE_RPM = 0x0C;
    static constexpr uint8_t PID_VEHICLE_SPEED = 0x0D;
    static constexpr uint8_t PID_COOLANT_TEMP = 0x05;
    static constexpr uint8_t PID_OIL_TEMP = 0x5C;
    static constexpr uint8_t PID_THROTTLE_POS = 0x11;

    /**
     * @brief Send OBD-II Mode 01 request
     * @param pid Parameter ID to request
     * @return true if request sent successfully
     */
    bool sendOBDRequest(uint8_t pid);

    /**
     * @brief Receive OBD-II response
     * @param pid Expected PID in response
     * @param data Buffer to store response data (up to 8 bytes)
     * @param dataLen Length of received data
     * @return true if valid response received
     */
    bool receiveOBDResponse(uint8_t pid, uint8_t* data, uint8_t& dataLen);

    /**
     * @brief Read engine RPM (PID 0x0C)
     * @return true if successful
     */
    bool readEngineRPM();

    /**
     * @brief Read vehicle speed (PID 0x0D)
     * @return true if successful
     */
    bool readVehicleSpeed();

    /**
     * @brief Read coolant temperature (PID 0x05)
     * @return true if successful
     */
    bool readCoolantTemp();

    /**
     * @brief Read oil temperature (PID 0x5C)
     * @return true if successful
     */
    bool readOilTemp();

    /**
     * @brief Read throttle position (PID 0x11)
     * @return true if successful
     */
    bool readThrottlePosition();

    /**
     * @brief Check if CAN data is stale
     * @return true if data should be marked invalid
     */
    bool isDataStale() const;
};

#endif // CAN_CONTROLLER_H
