#include "WebPortal.h"

WebPortal::WebPortal()
    : server(WEB_SERVER_PORT),
      ws(WEBSOCKET_PATH),
      lastCommandTime(0),
      lastTelemetryTime(0),
      otaInProgress(false),
      lastActivityTime(0) {
    currentCommand.hasCommand = false;
}

WebPortal::~WebPortal() {
    // Cleanup if needed
}

bool WebPortal::begin() {
    Serial.println("\n=== Initializing Web Portal ===");

    // Mount filesystem first
    if (!mountFilesystem()) {
        Serial.println("  ✗ Filesystem mount failed");
        return false;
    }

    // Setup WiFi Access Point
    if (!setupWiFiAP()) {
        Serial.println("  ✗ WiFi AP setup failed");
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
    Serial.println("  ✓ Web server started");
    Serial.printf("  ✓ Access portal at: http://%s\n", getAPIP().c_str());
    Serial.println("=== Web Portal Initialization Complete ===\n");

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

    String json = createTelemetryJSON(telemetry);
    ws.textAll(json);
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
    Serial.println("Setting up WiFi Access Point...");

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
        Serial.println("  ✗ Failed to start WiFi AP");
        return false;
    }

    Serial.printf("  ✓ WiFi AP started\n");
    Serial.printf("  ✓ SSID: %s\n", WIFI_AP_SSID);
    Serial.printf("  ✓ IP: %s\n", WiFi.softAPIP().toString().c_str());
    Serial.printf("  ✓ Max clients: %d\n", WIFI_AP_MAX_CLIENTS);

    return true;
}

void WebPortal::setupWebServer() {
    Serial.println("Setting up web server...");

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

    // Serve static files from SPIFFS
    server.serveStatic("/", FILESYSTEM, "/");

    // Handle 404 - redirect to main page for captive portal
    server.onNotFound([](AsyncWebServerRequest* request) {
        // Redirect to main page instead of showing 404
        request->redirect("/");
    });

    Serial.println("  ✓ Web server configured");
    Serial.println("  ✓ Captive portal endpoints added");
}

void WebPortal::setupWebSocket() {
    Serial.println("Setting up WebSocket...");

    // Attach WebSocket event handler (using lambda to call member function)
    ws.onEvent([this](AsyncWebSocket* server, AsyncWebSocketClient* client,
                      AwsEventType type, void* arg, uint8_t* data, size_t len) {
        this->onWebSocketEvent(server, client, type, arg, data, len);
    });

    // Add WebSocket handler to server
    server.addHandler(&ws);

    Serial.println("  ✓ WebSocket configured on /ws");
}

void WebPortal::setupOTA() {
    Serial.println("Setting up OTA updates...");

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
        Serial.println("\n[OTA] Starting update: " + type);
        otaInProgress = true;

        // Notify web clients
        sendResponse(true, "OTA update started");
    });

    ArduinoOTA.onEnd([this]() {
        Serial.println("\n[OTA] Update complete");
        otaInProgress = false;
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        static unsigned int lastPercent = 0;
        unsigned int percent = (progress / (total / 100));
        if (percent != lastPercent && percent % 10 == 0) {
            Serial.printf("[OTA] Progress: %u%%\n", percent);
            lastPercent = percent;
        }
    });

    ArduinoOTA.onError([this](ota_error_t error) {
        Serial.printf("[OTA] Error[%u]: ", error);
        String errorMsg;
        if (error == OTA_AUTH_ERROR) errorMsg = "Auth Failed";
        else if (error == OTA_BEGIN_ERROR) errorMsg = "Begin Failed";
        else if (error == OTA_CONNECT_ERROR) errorMsg = "Connect Failed";
        else if (error == OTA_RECEIVE_ERROR) errorMsg = "Receive Failed";
        else if (error == OTA_END_ERROR) errorMsg = "End Failed";
        else errorMsg = "Unknown Error";

        Serial.println(errorMsg);
        otaInProgress = false;

        // Notify web clients
        sendResponse(false, "OTA failed: " + errorMsg);
    });

    ArduinoOTA.begin();
    Serial.printf("  ✓ OTA configured (hostname: %s)\n", OTA_HOSTNAME);
}

void WebPortal::setupDNS() {
    Serial.println("Setting up DNS server for captive portal...");

    // Start DNS server on port 53, redirect all requests to AP IP
    // This makes the captive portal work by catching all DNS queries
    dnsServer.start(53, "*", WiFi.softAPIP());

    Serial.println("  ✓ DNS server started (captive portal enabled)");
}

bool WebPortal::mountFilesystem() {
    Serial.println("Mounting SPIFFS filesystem...");

    if (!FILESYSTEM.begin(false)) {  // false = don't format on fail
        Serial.println("  ✗ SPIFFS mount failed");
        Serial.println("  ! Run 'pio run -t uploadfs' to upload filesystem");
        return false;
    }

    Serial.println("  ✓ SPIFFS mounted");

    // Check if index.html exists
    if (!FILESYSTEM.exists("/index.html")) {
        Serial.println("  ⚠ Warning: /index.html not found in filesystem");
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
            Serial.printf("[WebSocket] Client #%u connected from %s\n",
                         client->id(), client->remoteIP().toString().c_str());
            lastActivityTime = millis();
            break;

        case WS_EVT_DISCONNECT:
            Serial.printf("[WebSocket] Client #%u disconnected\n", client->id());
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
                    Serial.printf("[WebSocket] Command received: %s\n", currentCommand.cmd.c_str());
                } else {
                    Serial.println("[WebSocket] Failed to parse command");
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
        Serial.printf("[WebSocket] JSON parse error: %s\n", error.c_str());
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
        if (doc["value"].is<const char*>()) {
            currentCommand.strValue = doc["value"].as<String>();
            currentCommand.floatValue = 0.0f;
        } else if (doc["value"].is<float>() || doc["value"].is<int>()) {
            currentCommand.floatValue = doc["value"].as<float>();
            currentCommand.strValue = "";
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
