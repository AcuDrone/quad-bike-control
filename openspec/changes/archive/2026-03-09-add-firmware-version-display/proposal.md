# Proposal: add-firmware-version-display

## Why

Users and developers need to quickly identify which firmware version is running on the ESP32 when accessing the web portal. This is essential for:
- **Troubleshooting**: Confirming the correct firmware is deployed
- **Development**: Verifying successful OTA updates or manual uploads
- **Documentation**: Matching behavior to specific release versions
- **Support**: Providing version info when reporting issues

Currently, there is no way to see the firmware version from the web interface - users must check serial output or source code.

## What Changes

Add firmware version display to the web portal status bar that reads from a centralized version constant.

### Scope

**In scope:**
- Define firmware version constant in Constants.h
- Add version field to telemetry data structure
- Include version in WebSocket telemetry broadcasts
- Display version in web portal status bar (always visible)

**Out of scope:**
- Automatic version numbering from git tags or build system
- Version history or changelog display
- Semantic versioning validation
- Minimum version compatibility checks

### Affected Components

1. **Constants.h** - Add `FIRMWARE_VERSION` constant
2. **WebPortal.h** - Add `firmware_version` to Telemetry struct
3. **TelemetryManager.cpp** - Include version in telemetry collection
4. **WebPortal.cpp** - Serialize version to JSON
5. **index.html** - Display version in status bar UI

## Implementation Approach

Follow the existing telemetry data flow pattern:

1. **Constants.h**: Define version string (e.g., "1.0.0" or "2026-03-09")
2. **Backend**: Add version to telemetry struct and populate it during collection
3. **Serialization**: Include version in JSON telemetry message
4. **Frontend**: Add status bar item that updates with version from WebSocket

**Display location**: Status bar (next to WebSocket connection status)
- Always visible at top of page
- Follows existing `.status-item` design pattern
- Updates once on initial connection (version doesn't change during runtime)

## Trade-offs

### Version Format Options

**Option A: Semantic version (e.g., "1.0.0")** ✅ **Recommended**
- Pros: Industry standard, clear release progression
- Cons: Requires manual updates
- Use case: Production releases

**Option B: Date-based (e.g., "2026-03-09")**
- Pros: Automatically indicates build recency
- Cons: Doesn't convey feature changes
- Use case: Development builds

**Option C: Hybrid (e.g., "1.0.0-20260309")**
- Pros: Both release and build date info
- Cons: Longer string, more maintenance
- Use case: Both production and development

**Decision**: Start with Option A (semantic version) as it's simplest and most flexible. Can be enhanced later with build date if needed.

### Display Location Options

**Option A: Status bar** ✅ **Recommended**
- Pros: Always visible, prominent, follows existing pattern
- Cons: Takes up status bar space
- Chosen because: Version is system-critical info like connection status

**Option B: Footer**
- Pros: Out of the way, traditional placement
- Cons: May not be visible if page scrolls, less prominent
- Not chosen because: Current design has no footer

**Option C: Settings/About dialog**
- Pros: Doesn't use screen real estate
- Cons: Hidden, requires extra clicks
- Not chosen because: Defeats purpose of quick identification

## Risks

| Risk | Severity | Mitigation |
|------|----------|------------|
| Forgot to update version constant | Low | Document in release process, future: automate |
| Version string too long breaks UI | Low | CSS handles overflow, keep version strings concise |
| WebSocket payload size increase | Very Low | String adds ~20 bytes (negligible) |

## Open Questions

None - implementation approach is straightforward and follows existing patterns.
