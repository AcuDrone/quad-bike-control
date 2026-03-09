# Proposal: enhance-webportal-telemetry-and-controls

## Summary
Enhance web portal with comprehensive telemetry display and brake control, including SBUS channel values, CAN bus vehicle data, brake actuator control slider, and gear transition status indicators.

## Problem
The current web portal has limited telemetry visibility and control capabilities:
1. **SBUS visibility**: Only shows active/inactive status, not actual channel values (steering, throttle, gear, brake inputs from ArduPilot)
2. **CAN data invisibility**: CAN bus data (engine RPM, speed, temperatures) is collected but not displayed
3. **Missing brake control**: No manual brake actuator control from web interface (only steering/throttle)
4. **Unclear gear transitions**: When switching gears, no visual feedback during transition (may briefly show neutral during physical actuator movement)

## Proposed Solution
Extend web portal telemetry and control capabilities:

### 1. SBUS Channel Display
- Add telemetry card showing all SBUS channel values (raw microseconds and normalized percentages)
- Display key mapped channels: steering, throttle, gear selector, brake
- Show signal quality metrics: frame rate, error rate, signal age
- Update in real-time (5 Hz) when SBUS is active

### 2. CAN Bus Vehicle Data Display
- Add telemetry card showing CAN bus data already collected by backend:
  - Engine RPM (0-16383)
  - Vehicle speed (km/h)
  - Coolant temperature (°C)
  - Oil temperature (°C)
  - Throttle position (%)
  - CAN connection status and data age
- Show "disconnected" state when CAN data is stale or invalid
- Color-code temperature warnings (green: normal, yellow: warm, red: hot)

### 3. Brake Actuator Control Slider
- Add brake slider to Manual Control card (alongside steering/throttle)
- Range: 0-100% brake application
- Sends `set_brake` WebSocket command
- Disabled when SBUS control is active (same as other controls)
- Backend: Add `set_brake` command handler in VehicleController

### 4. Gear Transition State Indicator
- Track gear transition state in TransmissionController:
  - `STABLE`: at target gear position
  - `SWITCHING`: position control active, moving to target gear
- Include `gear_switching` boolean in telemetry broadcast
- Web UI: When switching is true, show visual indicator (e.g., "⚙️ Switching to H...")
- Prevent UI from flashing neutral during transitions

## Benefits
- **Improved visibility**: Operators can see all input sources (SBUS channels, CAN data)
- **Better diagnostics**: Real-time visibility of vehicle sensors helps troubleshooting
- **Complete control**: Brake actuator control completes the web control interface
- **Better UX**: Gear transition indicators provide clearer feedback during gear changes

## Affected Components
- **Frontend**: `data/index.html` - Add SBUS/CAN cards, brake slider, gear transition UI
- **Backend**: `TelemetryManager` - Add SBUS channel data to telemetry
- **Backend**: `TransmissionController` - Track gear switching state
- **Backend**: `VehicleController` - Add brake control command handler
- **Backend**: `WebPortal::Telemetry` struct - Add SBUS channels and gear switching fields

## Spec Deltas
- **web-telemetry**: MODIFIED - Add SBUS channels, CAN display, gear switching state
- **web-control**: MODIFIED - Add brake control slider command

## Out of Scope
- Historical telemetry logging/graphing (future enhancement)
- Configurable CAN message IDs from web UI (use Constants.h)
- SBUS channel remapping from web UI (use Constants.h)
- Brake position feedback (no sensor available yet)

## Risks and Mitigations
| Risk | Mitigation |
|------|------------|
| Increased telemetry size may impact WebSocket performance | Validate that message size stays under 1KB, test with 5 clients |
| Brake control adds another safety-critical command | Enforce same SBUS priority as steering/throttle, add validation |
| Gear switching detection may have false positives | Use positionControlActive flag from BTS7960Controller (reliable) |

## Open Questions
1. Should SBUS display show all 16 channels or only mapped channels (Ch1-4)?
   - **Proposed**: Show all 16 for diagnostics, highlight mapped channels
2. Should brake control have a safety confirmation dialog?
   - **Proposed**: No dialog (consistent with steering/throttle), but visual feedback
3. What temperature thresholds for CAN data color coding?
   - **Proposed**: Coolant: <90°C green, 90-105°C yellow, >105°C red
   - **Proposed**: Oil: <110°C green, 110-130°C yellow, >130°C red
