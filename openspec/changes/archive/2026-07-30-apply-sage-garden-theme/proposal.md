# Change: Apply Sage Garden dark theme to the web control UI

## Why
The web control interface (`data/index.html`) uses an ad-hoc blue-gradient palette with dozens of hardcoded color literals scattered across the `<style>` block, inline `style="..."` attributes, and JavaScript-set colors. This makes the UI look dated and inconsistent, and makes any future restyle error-prone. Adopting a single, coherent design theme — the "Sage Garden" dark palette from tweakcn — gives the control UI a modern, consistent look and centralizes color decisions in CSS custom properties.

## What Changes
- Define the Sage Garden **dark** palette as CSS custom properties in a `:root` block at the top of the `<style>` section in `data/index.html` (dark-only; no light mode, no `prefers-color-scheme` switching).
- Refactor the existing hardcoded color literals (in the `<style>` block, in inline `style="..."` attributes, and in JavaScript that sets `.style.color`) to reference the new `var(--token)` variables.
- Preserve all layout, structure, sizing, and behavior. This is a color/token-only restyle — no layout redesign, no DOM changes, no logic changes.
- Preserve clearly distinguishable semantic status colors (OK / connected, warning / failsafe, error / fault) required for safe operation of a vehicle control UI.
- Use system font-stack fallbacks only. The theme's named fonts (Antic, Signifier, JetBrains Mono) MUST NOT be loaded from Google Fonts or any CDN, because the ESP32 serves the entire UI from LittleFS with no internet access.

## Impact
- Affected specs: `web-server` (owns the "Serve static HTML page" capability; a new requirement describes the served page's visual theme).
- Affected code: `data/index.html` only (the entire single-file web UI). No C++/firmware source changes.
- Deployment: reflash the filesystem image with `pio run -t uploadfs`, then visually verify on a phone connected to the `QuadBike-Control` AP.
