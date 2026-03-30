#include "WebPortal.h"
#include "Debug.h"

WebPortal::WebPortal()
    : server(WEB_SERVER_PORT),
      ws(WEBSOCKET_PATH),
      lastCommandTime(0),
      lastTelemetryTime(0),
      otaInProgress(false),
      otaUpdateType(U_FLASH),
      lastActivityTime(0) {
    currentCommand.hasCommand = false;
}

WebPortal::~WebPortal() {
    // Cleanup if needed
}

bool WebPortal::begin() {
    Debug::printlnFeature(DebugFeature::WEB,"\n=== Initializing Web Portal ===");

    // Mount filesystem first
    if (!mountFilesystem()) {
        Debug::printlnFeature(DebugFeature::WEB,"  ✗ Filesystem mount failed");
        return false;
    }

    // Setup WiFi Access Point
    if (!setupWiFiAP()) {
        Debug::printlnFeature(DebugFeature::WEB,"  ✗ WiFi AP setup failed");
        return false;
    }

    // Setup WebSocket
    setupWebSocket();

    // Setup Web Server
    setupWebServer();

    // Setup OTA
    setupOTA();

    // Setup DNS server for captive portal
    setupDNS();

    // Start web server
    server.begin();
    Debug::printlnFeature(DebugFeature::WEB,"  ✓ Web server started");
    Debug::printfFeature(DebugFeature::WEB,"  ✓ Access portal at: http://%s\n", getAPIP().c_str());
    Debug::printlnFeature(DebugFeature::WEB,"=== Web Portal Initialization Complete ===\n");

    return true;
}

void WebPortal::update() {
    // Process DNS requests for captive portal
    dnsServer.processNextRequest();

    // Handle OTA updates
    ArduinoOTA.handle();

    // Check for command timeout (web control inactive)
    if (currentCommand.hasCommand) {
        if (millis() - lastCommandTime > WEB_COMMAND_TIMEOUT) {
            // Command timed out
            currentCommand.hasCommand = false;
        }
    }

    // Cleanup WebSocket clients periodically
    ws.cleanupClients();
}

bool WebPortal::hasActiveControl() {
    if (!currentCommand.hasCommand) {
        return false;
    }

    // Check if command is still fresh (within timeout)
    return (millis() - lastCommandTime) < WEB_COMMAND_TIMEOUT;
}

WebPortal::WebCommand WebPortal::getCommand() {
    return currentCommand;
}

void WebPortal::clearCommand() {
    currentCommand.hasCommand = false;
}

void WebPortal::broadcastTelemetry(const Telemetry& telemetry) {
    // Rate limit telemetry broadcasts (5 Hz = 200ms interval)
    if (millis() - lastTelemetryTime < TELEMETRY_INTERVAL) {
        return;
    }

    // Skip if no clients connected
    if (ws.count() == 0) {
        lastTelemetryTime = millis();
        return;
    }

    // Check if WebSocket queue has space (prevent overflow)
    // AsyncWebSocket has internal queue limit, skip broadcast if clients are lagging
    if (ws.availableForWriteAll()) {
        String json = createTelemetryJSON(telemetry);
        ws.textAll(json);
    }

    lastTelemetryTime = millis();
}

void WebPortal::sendResponse(bool success, const String& message) {
    String json = createResponseJSON(success, message);
    ws.textAll(json);
}

uint8_t WebPortal::getClientCount() {
    return ws.count();
}

bool WebPortal::isAPActive() {
    return WiFi.getMode() == WIFI_AP || WiFi.getMode() == WIFI_AP_STA;
}

String WebPortal::getAPIP() {
    return WiFi.softAPIP().toString();
}

// ============================================================================
// PRIVATE METHODS - SETUP
// ============================================================================

bool WebPortal::setupWiFiAP() {
    Debug::printlnFeature(DebugFeature::WEB,"Setting up WiFi Access Point...");

    // Configure Access Point
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(WIFI_AP_IP, WIFI_AP_GATEWAY, WIFI_AP_SUBNET);

    bool success = false;
    if (strlen(WIFI_AP_PASSWORD) > 0) {
        // With password
        success = WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD, WIFI_AP_CHANNEL, false, WIFI_AP_MAX_CLIENTS);
    } else {
        // Open network (no password)
        success = WiFi.softAP(WIFI_AP_SSID, NULL, WIFI_AP_CHANNEL, false, WIFI_AP_MAX_CLIENTS);
    }

    if (!success) {
        Debug::printlnFeature(DebugFeature::WEB,"  ✗ Failed to start WiFi AP");
        return false;
    }

    Debug::printfFeature(DebugFeature::WEB,"  ✓ WiFi AP started\n");
    Debug::printfFeature(DebugFeature::WEB,"  ✓ SSID: %s\n", WIFI_AP_SSID);
    Debug::printfFeature(DebugFeature::WEB,"  ✓ IP: %s\n", WiFi.softAPIP().toString().c_str());
    Debug::printfFeature(DebugFeature::WEB,"  ✓ Max clients: %d\n", WIFI_AP_MAX_CLIENTS);

    return true;
}

void WebPortal::setupWebServer() {
    Debug::printlnFeature(DebugFeature::WEB,"Setting up web server...");

    // Serve index.html from SPIFFS
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(FILESYSTEM, "/index.html", "text/html");
    });

    // Captive portal detection endpoints for different operating systems
    // Android captive portal check
    server.on("/generate_204", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->redirect("/");
    });

    // iOS/macOS captive portal check
    server.on("/hotspot-detect.html", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->redirect("/");
    });

    // Windows captive portal check
    server.on("/connecttest.txt", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->redirect("/");
    });

    // Firefox captive portal check
    server.on("/success.txt", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->redirect("/");
    });

    // Canonical captive portal check
    server.on("/canonical.html", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->redirect("/");
    });

    // Debug control API - GET current debug state
    server.on("/api/debug", HTTP_GET, [](AsyncWebServerRequest* request) {
        StaticJsonDocument<256> doc;
        doc["enabled"] = Debug::isEnabled();

        String response;
        serializeJson(doc, response);
        request->send(200, "application/json", response);
    });

    // Debug control API - POST to toggle debug state
    server.on("/api/debug", HTTP_POST, [](AsyncWebServerRequest* request) {}, nullptr,
        [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
            // Parse JSON body
            StaticJsonDocument<256> doc;
            DeserializationError error = deserializeJson(doc, data, len);

            if (error) {
                request->send(400, "application/json", "{\"success\":false,\"message\":\"Invalid JSON\"}");
                return;
            }

            if (!doc.containsKey("enabled")) {
                request->send(400, "application/json", "{\"success\":false,\"message\":\"Missing 'enabled' field\"}");
                return;
            }

            bool enabled = doc["enabled"];
            Debug::setEnabled(enabled);

            StaticJsonDocument<256> response;
            response["success"] = true;
            response["enabled"] = enabled;
            response["message"] = enabled ? "Debug output enabled" : "Debug output disabled";

            String responseStr;
            serializeJson(response, responseStr);
            request->send(200, "application/json", responseStr);
        }
    );

    // OTA firmware update endpoint
    server.on("/update", HTTP_POST,
        // Handler when upload finishes
        [this](AsyncWebServerRequest* request) {
            bool success = !Update.hasError();

            if (success) {
                Debug::printlnFeature(DebugFeature::WEB,"[WEB OTA] Update successful, rebooting...");
                request->send(200, "text/plain", "OK");

                // Reboot after short delay to allow response to be sent
                delay(1000);
                ESP.restart();
            } else {
                String error = "Update failed: ";
                error += Update.errorString();
                Debug::printfFeature(DebugFeature::WEB,"[WEB OTA] %s\n", error.c_str());
                request->send(500, "text/plain", error);
            }

            otaInProgress = false;
        },
        // Upload handler for data chunks
        [this](AsyncWebServerRequest* request, const String& filename, size_t index, uint8_t* data, size_t len, bool final) {
            // Start of upload
            if (index == 0) {
                // Determine update type from query parameter or filename
                otaUpdateType = U_FLASH;  // Default: firmware update
                String updateTypeName = "firmware";

                // Check for ?type=spiffs query parameter
                if (request->hasParam("type")) {
                    String type = request->getParam("type")->value();
                    if (type == "spiffs" || type == "filesystem") {
                        otaUpdateType = U_SPIFFS;
                        updateTypeName = "SPIFFS";
                        // Unmount filesystem before update
                        FILESYSTEM.end();
                    }
                }
                // Or detect from filename
                else if (filename.indexOf("spiffs") >= 0 || filename.indexOf("filesystem") >= 0) {
                    otaUpdateType = U_SPIFFS;
                    updateTypeName = "SPIFFS";
                    FILESYSTEM.end();
                }

                Debug::printfFeature(DebugFeature::WEB,"[WEB OTA] Starting %s update: %s\n", updateTypeName.c_str(), filename.c_str());
                Debug::printfFeature(DebugFeature::WEB,"[WEB OTA] File size: %zu bytes\n", request->contentLength());
                Debug::printfFeature(DebugFeature::WEB,"[WEB OTA] Free heap before update: %u bytes\n", ESP.getFreeHeap());

                otaInProgress = true;

                // Begin OTA update with specified type and size
                size_t updateSize;
                if (otaUpdateType == U_SPIFFS) {
                    // For SPIFFS, use UPDATE_SIZE_UNKNOWN to let Update library determine size
                    updateSize = UPDATE_SIZE_UNKNOWN;
                    Debug::printlnFeature(DebugFeature::WEB,"[WEB OTA] Using UPDATE_SIZE_UNKNOWN for SPIFFS");
                } else {
                    // For firmware, use actual content length
                    updateSize = request->contentLength();
                    if (updateSize == 0) {
                        updateSize = UPDATE_SIZE_UNKNOWN;
                    }
                }

                if (!Update.begin(updateSize, otaUpdateType)) {
                    Debug::printfFeature(DebugFeature::WEB,"[WEB OTA] Begin failed: %s\n", Update.errorString());
                    Debug::printfFeature(DebugFeature::WEB,"[WEB OTA] Requested size: %zu, Update type: %d\n", updateSize, otaUpdateType);
                    otaInProgress = false;

                    // Remount SPIFFS if it was unmounted for failed SPIFFS update
                    if (otaUpdateType == U_SPIFFS) {
                        Debug::printlnFeature(DebugFeature::WEB,"[WEB OTA] Remounting SPIFFS after failed update...");
                        FILESYSTEM.begin(false);
                    }
                    return;
                }
            }

            // Write firmware/filesystem chunk
            if (len) {
                size_t written = Update.write(data, len);
                if (written != len) {
                    Debug::printfFeature(DebugFeature::WEB,"[WEB OTA] Write failed at %zu bytes (wrote %zu of %zu bytes)\n", index, written, len);
                    Debug::printfFeature(DebugFeature::WEB,"[WEB OTA] Error: %s\n", Update.errorString());
                    Debug::printfFeature(DebugFeature::WEB,"[WEB OTA] Free heap: %u bytes\n", ESP.getFreeHeap());
                    return;
                }

                // Log progress every 64KB
                if (index % (64 * 1024) == 0 && index > 0) {
                    Debug::printfFeature(DebugFeature::WEB,"[WEB OTA] Progress: %zu bytes written (free heap: %u)\n", index + len, ESP.getFreeHeap());
                }
            }

            // End of upload
            if (final) {
                if (Update.end(true)) {
                    Debug::printfFeature(DebugFeature::WEB,"[WEB OTA] Update complete: %zu bytes\n", index + len);
                } else {
                    Debug::printfFeature(DebugFeature::WEB,"[WEB OTA] End failed: %s\n", Update.errorString());

                    // Remount SPIFFS if it was unmounted for failed SPIFFS update
                    if (otaUpdateType == U_SPIFFS) {
                        Debug::printlnFeature(DebugFeature::WEB,"[WEB OTA] Remounting SPIFFS after failed update...");
                        FILESYSTEM.begin(false);
                    }
                }
            }
        }
    );

    // Serve static files from SPIFFS
    server.serveStatic("/", FILESYSTEM, "/");

    // Handle 404 - redirect to main page for captive portal
    server.onNotFound([](AsyncWebServerRequest* request) {
        // Redirect to main page instead of showing 404
        request->redirect("/");
    });

    Debug::printlnFeature(DebugFeature::WEB,"  ✓ Web server configured");
    Debug::printlnFeature(DebugFeature::WEB,"  ✓ Captive portal endpoints added");
}

void WebPortal::setupWebSocket() {
    Debug::printlnFeature(DebugFeature::WEB,"Setting up WebSocket...");

    // Attach WebSocket event handler (using lambda to call member function)
    ws.onEvent([this](AsyncWebSocket* server, AsyncWebSocketClient* client,
                      AwsEventType type, void* arg, uint8_t* data, size_t len) {
        this->onWebSocketEvent(server, client, type, arg, data, len);
    });

    // Add WebSocket handler to server
    server.addHandler(&ws);

    Debug::printlnFeature(DebugFeature::WEB,"  ✓ WebSocket configured on /ws");
}

void WebPortal::setupOTA() {
    Debug::printlnFeature(DebugFeature::WEB,"Setting up OTA updates...");

    // Set OTA hostname
    ArduinoOTA.setHostname(OTA_HOSTNAME);

    // Set OTA password if configured
    if (strlen(OTA_PASSWORD) > 0) {
        ArduinoOTA.setPassword(OTA_PASSWORD);
    }

    // OTA callbacks
    ArduinoOTA.onStart([this]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH) {
            type = "sketch";
        } else {  // U_SPIFFS or U_FILESYSTEM
            type = "filesystem";
            // Unmount filesystem before OTA
            FILESYSTEM.end();
        }
        Debug::printlnFeature(DebugFeature::WEB,"\n[OTA] Starting update: " + type);
        otaInProgress = true;

        // Notify web clients
        sendResponse(true, "OTA update started");
    });

    ArduinoOTA.onEnd([this]() {
        Debug::printlnFeature(DebugFeature::WEB,"\n[OTA] Update complete");
        otaInProgress = false;
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        static unsigned int lastPercent = 0;
        unsigned int percent = (progress / (total / 100));
        if (percent != lastPercent && percent % 10 == 0) {
            Debug::printfFeature(DebugFeature::WEB,"[OTA] Progress: %u%%\n", percent);
            lastPercent = percent;
        }
    });

    ArduinoOTA.onError([this](ota_error_t error) {
        Debug::printfFeature(DebugFeature::WEB,"[OTA] Error[%u]: ", error);
        String errorMsg;
        if (error == OTA_AUTH_ERROR) errorMsg = "Auth Failed";
        else if (error == OTA_BEGIN_ERROR) errorMsg = "Begin Failed";
        else if (error == OTA_CONNECT_ERROR) errorMsg = "Connect Failed";
        else if (error == OTA_RECEIVE_ERROR) errorMsg = "Receive Failed";
        else if (error == OTA_END_ERROR) errorMsg = "End Failed";
        else errorMsg = "Unknown Error";

        Debug::printlnFeature(DebugFeature::WEB,errorMsg);
        otaInProgress = false;

        // Notify web clients
        sendResponse(false, "OTA failed: " + errorMsg);
    });

    ArduinoOTA.begin();
    Debug::printfFeature(DebugFeature::WEB,"  ✓ OTA configured (hostname: %s)\n", OTA_HOSTNAME);
}

void WebPortal::setupDNS() {
    Debug::printlnFeature(DebugFeature::WEB,"Setting up DNS server for captive portal...");

    // Start DNS server on port 53, redirect all requests to AP IP
    // This makes the captive portal work by catching all DNS queries
    dnsServer.start(53, "*", WiFi.softAPIP());

    Debug::printlnFeature(DebugFeature::WEB,"  ✓ DNS server started (captive portal enabled)");
}

bool WebPortal::mountFilesystem() {
    Debug::printlnFeature(DebugFeature::WEB,"Mounting SPIFFS filesystem...");

    if (!FILESYSTEM.begin(false)) {  // false = don't format on fail
        Debug::printlnFeature(DebugFeature::WEB,"  ✗ SPIFFS mount failed");
        Debug::printlnFeature(DebugFeature::WEB,"  ! Run 'pio run -t uploadfs' to upload filesystem");
        return false;
    }

    Debug::printlnFeature(DebugFeature::WEB,"  ✓ SPIFFS mounted");

    // Check if index.html exists
    if (!FILESYSTEM.exists("/index.html")) {
        Debug::printlnFeature(DebugFeature::WEB,"  ⚠ Warning: /index.html not found in filesystem");
    }

    return true;
}

// ============================================================================
// PRIVATE METHODS - WEBSOCKET HANDLERS
// ============================================================================

void WebPortal::onWebSocketEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                                  AwsEventType type, void* arg, uint8_t* data, size_t len) {
    switch (type) {
        case WS_EVT_CONNECT:
            Debug::printfFeature(DebugFeature::WEB,"[WebSocket] Client #%     connected from %s\n",
                         client->id(), client->remoteIP().toString().c_str());
            lastActivityTime = millis();
            break;

        case WS_EVT_DISCONNECT:
            Debug::printfFeature(DebugFeature::WEB,"[WebSocket] Client #%u disconnected\n", client->id());
            // If this was the controlling client, clear command
            currentCommand.hasCommand = false;
            break;

        case WS_EVT_DATA: {
            AwsFrameInfo* info = (AwsFrameInfo*)arg;
            if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
                // Complete text frame received
                if (parseWebCommand(data, len)) {
                    lastCommandTime = millis();
                    lastActivityTime = millis();
                    Debug::printfFeature(DebugFeature::WEB,"[WebSocket] Command received: %s\n", currentCommand.cmd.c_str());
                } else {
                    Debug::printlnFeature(DebugFeature::WEB,"[WebSocket] Failed to parse command");
                    sendResponse(false, "Invalid command format");
                }
            }
            break;
        }

        case WS_EVT_PONG:
        case WS_EVT_ERROR:
            break;
    }
}

bool WebPortal::parseWebCommand(uint8_t* data, size_t len) {
    // Parse JSON command
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, data, len);

    if (error) {
        Debug::printfFeature(DebugFeature::WEB,"[WebSocket] JSON parse error: %s\n", error.c_str());
        return false;
    }

    // Extract command fields
    if (!doc.containsKey("cmd")) {
        return false;
    }

    currentCommand.cmd = doc["cmd"].as<String>();
    currentCommand.hasCommand = true;

    // Extract value based on command type
    if (doc.containsKey("value")) {
        if (doc["value"].is<bool>()) {
            currentCommand.boolValue = doc["value"].as<bool>();
            currentCommand.strValue = "";
            currentCommand.floatValue = 0.0f;
        } else if (doc["value"].is<const char*>()) {
            currentCommand.strValue = doc["value"].as<String>();
            currentCommand.floatValue = 0.0f;
            currentCommand.boolValue = false;
        } else if (doc["value"].is<float>() || doc["value"].is<int>()) {
            currentCommand.floatValue = doc["value"].as<float>();
            currentCommand.strValue = "";
            currentCommand.boolValue = false;
        }
    } else {
        return false;
    }

    return true;
}

bool WebPortal::validateCommand(const WebCommand& cmd, InputSource inputSource) {
    // Only allow web commands when input source is WEB (not SBUS or FAILSAFE)
    if (inputSource != InputSource::WEB) {
        return false;
    }

    // Validate command type
    if (cmd.cmd != "set_gear" && cmd.cmd != "set_steering" && cmd.cmd != "set_throttle") {
        return false;
    }

    // Validate ranges
    if (cmd.cmd == "set_steering") {
        if (cmd.floatValue < -100.0f || cmd.floatValue > 100.0f) {
            return false;
        }
    } else if (cmd.cmd == "set_throttle") {
        if (cmd.floatValue < 0.0f || cmd.floatValue > 100.0f) {
            return false;
        }
    } else if (cmd.cmd == "set_gear") {
        if (cmd.strValue != "R" && cmd.strValue != "N" &&
            cmd.strValue != "L" && cmd.strValue != "H") {
            return false;
        }
    }

    return true;
}

// ============================================================================
// PRIVATE METHODS - JSON FORMATTING
// ============================================================================

String WebPortal::createTelemetryJSON(const Telemetry& telemetry) {
    StaticJsonDocument<1024> doc;  // Increased size for CAN data + SBUS channels

    doc["timestamp"] = telemetry.timestamp;
    doc["gear"] = telemetry.gear;
    doc["hall_position"] = telemetry.hall_position;
    doc["brake_pct"] = telemetry.brake_pct;
    doc["throttle_angle"] = telemetry.throttle_angle;
    doc["steering_angle"] = telemetry.steering_angle;
    doc["input_source"] = telemetry.input_source;
    doc["sbus_active"] = telemetry.sbus_active;

    // CAN bus vehicle data
    if (telemetry.can_status == "connected") {
        doc["engine_rpm"] = telemetry.engine_rpm;
        doc["vehicle_speed"] = telemetry.vehicle_speed;
        doc["coolant_temp"] = telemetry.coolant_temp;
        doc["oil_temp"] = telemetry.oil_temp;
        doc["throttle_position"] = telemetry.throttle_position;
        doc["fuel_level"] = telemetry.fuel_level;
    }
    doc["can_status"] = telemetry.can_status;
    doc["can_data_age"] = telemetry.can_data_age;

    // SBUS channel data
    JsonArray sbusChannels = doc.createNestedArray("sbus_channels");
    for (int i = 0; i < 16; i++) {
        sbusChannels.add(telemetry.sbus_channels[i]);
    }
    doc["sbus_frame_rate"] = telemetry.sbus_frame_rate;
    doc["sbus_error_rate"] = telemetry.sbus_error_rate;
    doc["sbus_signal_age"] = telemetry.sbus_signal_age;

    // Gear switching state
    doc["gear_switching"] = telemetry.gear_switching;

    // Ignition and lighting state
    doc["ignition_state"] = telemetry.ignition_state;
    doc["is_cranking"] = telemetry.is_cranking;
    doc["front_light_on"] = telemetry.front_light_on;

    // Firmware version
    doc["firmware_version"] = telemetry.firmware_version;

    String json;
    serializeJson(doc, json);
    return json;
}

String WebPortal::createResponseJSON(bool success, const String& message) {
    StaticJsonDocument<256> doc;

    doc["success"] = success;
    doc["message"] = message;

    String json;
    serializeJson(doc, json);
    return json;
}

bool WebPortal::isOTAInProgress() {
    return otaInProgress;
}
