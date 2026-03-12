## 1. Create Debug Utility Infrastructure
- [x] 1.1 Create include/Debug.h header with Debug class declaration
- [x] 1.2 Implement Debug::print(), Debug::println(), Debug::printf() methods
- [x] 1.3 Add runtime check against DEBUG_ENABLED in each method
- [x] 1.4 Support all Arduino printable types (String, char*, int, float, bool, etc.)
- [x] 1.5 Add NVS key for debug state persistence (NVS_KEY_DEBUG_ENABLED)

## 2. Implement Debug State Persistence
- [x] 2.1 Add saveDebugState() method to save DEBUG_ENABLED to NVS
- [x] 2.2 Add loadDebugState() method to restore DEBUG_ENABLED from NVS
- [x] 2.3 Call loadDebugState() in main.cpp during initialization
- [x] 2.4 Default to Constants.h DEBUG_ENABLED value if NVS not set

## 3. Add Web Portal Debug Control API
- [x] 3.1 Add POST /api/debug endpoint in WebPortal.cpp
- [x] 3.2 Parse JSON {"enabled": true/false} from request body
- [x] 3.3 Update DEBUG_ENABLED global variable
- [x] 3.4 Call saveDebugState() to persist to NVS
- [x] 3.5 Return JSON success response
- [x] 3.6 Add GET /api/debug endpoint to query current state
- [x] 3.7 Return JSON {"enabled": <boolean>} with current DEBUG_ENABLED value

## 4. Refactor Existing Serial Calls
- [x] 4.1 Replace Serial calls in src/main.cpp (30+ occurrences)
- [x] 4.2 Replace Serial calls in src/WebPortal.cpp (20+ occurrences)
- [x] 4.3 Replace Serial calls in src/RelayController.cpp (8 occurrences)
- [x] 4.4 Replace Serial calls in src/CANController.cpp
- [x] 4.5 Replace Serial calls in src/SBusInput.cpp
- [x] 4.6 Replace Serial calls in src/VehicleController.cpp
- [x] 4.7 Replace Serial calls in src/TransmissionController.cpp
- [x] 4.8 Replace Serial calls in src/BTS7960Controller.cpp
- [x] 4.9 Replace Serial calls in src/EncoderCounter.cpp
- [x] 4.10 Replace Serial calls in src/ServoController.cpp

## 5. Update Constants and Documentation
- [x] 5.1 Update Constants.h comment for DEBUG_ENABLED to reference Debug utility
- [x] 5.2 Add extern declaration for DEBUG_ENABLED global variable if needed
- [x] 5.3 Verify all files include Debug.h where needed

## 6. Testing and Validation
- [ ] 6.1 Test Debug::print() with debug enabled - verify output appears
- [ ] 6.2 Test Debug::print() with debug disabled - verify no output
- [ ] 6.3 Test POST /api/debug with {"enabled": true} - verify debug turns on
- [ ] 6.4 Test POST /api/debug with {"enabled": false} - verify debug turns off
- [ ] 6.5 Test GET /api/debug - verify correct state returned
- [ ] 6.6 Test NVS persistence - toggle debug, reboot, verify state restored
- [ ] 6.7 Compile and verify binary size impact is minimal
- [ ] 6.8 Test all refactored modules for correct debug output
- [ ] 6.9 Verify no Serial calls remain except critical bootloader messages
