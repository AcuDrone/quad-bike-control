## ADDED Requirements

### Requirement: Web UI Visual Theme
The served control interface (`data/index.html`) SHALL present a single, coherent dark visual theme based on the "Sage Garden" palette, with all colors defined as CSS custom properties in a `:root` block and referenced via `var()` throughout the page. The theme SHALL NOT depend on any external network resource (no CDN, no remote fonts). Safety-relevant status colors SHALL remain clearly distinguishable.

#### Scenario: Dark theme applied as the single theme
- **WHEN** the page loads in a client browser
- **THEN** the page renders using the Sage Garden dark palette (dark background, light foreground, sage-green primary)
- **AND** no light-mode variant and no `prefers-color-scheme` switching is present
- **AND** colors are sourced from CSS custom properties defined in `:root`

#### Scenario: Distinguishable safety status colors
- **WHEN** the UI shows connection, control-source, and fault states
- **THEN** OK / connected states render in a green tone distinct from warning and error
- **AND** failsafe / warning states render in an amber tone distinct from OK and error
- **AND** sensor-fault / disconnected / error states render in a red tone distinct from OK and warning

#### Scenario: No external font or network dependency
- **WHEN** the page is served from LittleFS while the device has no internet access
- **THEN** the UI styles and fonts load entirely from the served page with no requests to Google Fonts or any CDN
- **AND** text renders using a system font stack fallback
