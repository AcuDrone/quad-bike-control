# Change: Add Ignition and Light Control to Web Portal

## Why
The RelayController hardware and software infrastructure is already implemented for controlling ignition states (OFF/ACC/IGNITION/CRANKING) and front light, but there is no web interface to control these systems. Users need convenient web-based buttons to start the engine, control ignition states, and toggle the front light without relying solely on S-bus input.

## What Changes
- Add web interface controls for ignition (OFF, ACC, IGNITION, START buttons) and front light (toggle switch)
- Add WebSocket commands for ignition state control (set_ignition) and light control (set_light)
- Add ignition state and light status to telemetry broadcasts
- Implement safety interlocks: require brakes applied before ignition changes, prevent cranking if engine already running
- Create new "Vehicle Ignition & Lighting" card in web portal UI

## Impact
- Affected specs: web-control, vehicle-systems
- Affected code:
  - `src/WebPortal.cpp`: Add WebSocket command handlers, telemetry fields
  - `src/VehicleController.cpp`: Expose ignition and light control methods to web portal
  - `data/index.html`: Add new UI card with buttons and telemetry displays
- Safety-critical: Ignition control affects vehicle operation and must enforce safety constraints
