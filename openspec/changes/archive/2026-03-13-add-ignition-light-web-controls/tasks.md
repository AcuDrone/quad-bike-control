# Implementation Tasks

## 1. VehicleController Integration
- [x] 1.1 Add setIgnitionState() public method to VehicleController
- [x] 1.2 Add getIgnitionState() public method to VehicleController
- [x] 1.3 Add setFrontLight() public method to VehicleController
- [x] 1.4 Add getFrontLight() public method to VehicleController
- [x] 1.5 Implement safety interlock: check brake >= 20% before ignition power-on from OFF
- [x] 1.6 Implement safety interlock: check engine RPM < 1100 before allowing START
- [x] 1.7 Call RelayController.update() in VehicleController.update() with CAN engine RPM
- [x] 1.8 Add ignition and light state to diagnostic output

## 2. WebPortal WebSocket Commands
- [x] 2.1 Add set_ignition command handler in WebPortal::handleWebSocketMessage()
- [x] 2.2 Add set_light command handler in WebPortal::handleWebSocketMessage()
- [x] 2.3 Validate ignition state values (OFF/ACC/IGNITION/START) in command handler
- [x] 2.4 Validate light state values (boolean) in command handler
- [x] 2.5 Call VehicleController ignition/light methods from command handlers
- [x] 2.6 Send success/error responses via WebSocket
- [x] 2.7 Handle safety interlock errors (brake not applied, engine already running) with descriptive messages

## 3. WebPortal Telemetry Updates
- [x] 3.1 Add ignition_state field to telemetry JSON (string: OFF/ACC/IGNITION/CRANKING)
- [x] 3.2 Add is_cranking field to telemetry JSON (boolean)
- [x] 3.3 Add front_light_on field to telemetry JSON (boolean)
- [x] 3.4 Query VehicleController for ignition and light states in buildTelemetryJson()
- [x] 3.5 Ensure telemetry updates reflect real-time ignition and light changes

## 4. Web UI - New Ignition & Lighting Card
- [x] 4.1 Create new "Vehicle Ignition & Lighting" card in HTML after Manual Control card
- [x] 4.2 Add four ignition buttons: OFF, ACC, IGNITION, START with appropriate styling
- [x] 4.3 Add front light toggle switch with ON/OFF labels
- [x] 4.4 Add ignition state indicator in telemetry display
- [x] 4.5 Add light state indicator in telemetry display
- [x] 4.6 Style buttons with active/inactive states (active button highlighted in green)

## 5. Web UI - JavaScript Event Handlers
- [x] 5.1 Implement sendIgnitionCommand(state) function to send set_ignition WebSocket command
- [x] 5.2 Implement sendLightCommand(state) function to send set_light WebSocket command
- [x] 5.3 Add onclick handlers to all four ignition buttons
- [x] 5.4 Add onclick handler to light toggle switch
- [x] 5.5 Update button active states based on telemetry ignition_state field
- [x] 5.6 Update light toggle state based on telemetry front_light_on field
- [x] 5.7 Show visual feedback during command transmission

## 6. Web UI - Telemetry Display Updates
- [x] 6.1 Update updateTelemetry() JavaScript function to handle ignition_state field
- [x] 6.2 Update updateTelemetry() to handle is_cranking field (show animation during cranking)
- [x] 6.3 Update updateTelemetry() to handle front_light_on field
- [x] 6.4 Ensure ignition button highlighting updates in real-time
- [x] 6.5 Ensure light toggle reflects current state from telemetry

## 7. Safety Interlock Error Handling
- [x] 7.1 Display user-friendly error messages when brake not applied before ignition
- [x] 7.2 Display user-friendly error messages when attempting to crank running engine
- [x] 7.3 Add calibration status notification area for safety error messages
- [x] 7.4 Auto-dismiss error messages after 5 seconds

## 8. Testing and Validation
- [x] 8.1 Test ignition OFF button (verify relays turn off)
- [x] 8.2 Test ignition ACC button (verify RELAY2 on, RELAY1 off)
- [x] 8.3 Test ignition IGNITION button (verify both relays on)
- [x] 8.4 Test ignition START button (verify cranking starts, auto-stops after engine start or timeout)
- [x] 8.5 Test front light toggle (verify RELAY3 output changes)
- [x] 8.6 Test safety interlock: brake requirement before ignition power-on
- [x] 8.7 Test safety interlock: prevent cranking when engine running (simulate RPM > 1100)
- [x] 8.8 Test telemetry updates reflect ignition and light changes in real-time
- [x] 8.9 Test WebSocket command validation (invalid ignition states, invalid light values)
- [x] 8.10 Test UI button states update correctly based on telemetry
- [x] 8.11 Verify ignition controls work independently of S-bus status (always available)
- [x] 8.12 Test cranking auto-stop via engine RPM detection (when CAN available)
- [x] 8.13 Test cranking auto-stop via 5-second timeout
