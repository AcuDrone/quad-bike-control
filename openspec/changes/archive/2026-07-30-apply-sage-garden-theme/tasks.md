## 1. Define theme tokens
- [x] 1.1 Add a `:root` block at the top of the `<style>` section in `data/index.html` defining all Sage Garden dark tokens from design.md (`--background`, `--foreground`, `--card`, `--card-foreground`, `--popover`, `--primary`, `--primary-foreground`, `--secondary`, `--secondary-foreground`, `--muted`, `--muted-foreground`, `--accent`, `--accent-foreground`, `--destructive`, `--destructive-foreground`, `--border`, `--input`, `--ring`, `--radius`)
- [x] 1.2 Add semantic status tokens `--success` (= `--primary`), `--warning` (`#ff9800`), `--info` (`#2196f3`), `--danger` (= `--destructive`)
- [x] 1.3 Add font-stack variables using system fonts only (no remote `@font-face`, no Google Fonts / CDN `<link>`)

## 2. Refactor hardcoded colors to var() references (section by section)
- [x] 2.1 `body`, `header`, `h1`, `.container` — background/gradient → `--background`, text → `--foreground`
- [x] 2.2 `.card`, `.card h2`, `.status-bar`, `.status-item` (base + `.connected`/`.disconnected`/`.mav-active`/`.web-active`/`.failsafe`) → surfaces `--card`, borders `--border`, states `--success`/`--danger`/`--info`/`--warning`
- [x] 2.3 `.telemetry-grid`, `.telemetry-item`, `.telemetry-label`, `.telemetry-value`, `.gear-display` → `--card` / `--muted-foreground` / `--foreground`
- [x] 2.4 `.gear-btn` (base/hover/disabled/active), `.slider` + thumbs + disabled, `.progress-bar`, `.progress-fill` → `--primary`/`--primary-foreground`/`--input`/`--border`
- [x] 2.5 `.upload-btn`, `.file-input-label` (+ hover), `.ota-section`, `.warning` banner → `--primary` / `--info` / `--warning`
- [x] 2.6 Inline `style="..."` attributes on steering test / jog / nudge / capture buttons and status spans (`#607d8b`, `#795548`, `#2196F3`, `#ff9800`, `#fff`, `#4caf50`) → matching tokens
- [x] 2.7 JavaScript color assignments (`statusEl.style.color = '#4caf50'/'#f44336'`, throttle/steer calibration status setters) → `var(--success)` / `var(--danger)`

## 3. Verify semantic status colors
- [x] 3.1 Confirm connected/OK renders sage green, disconnected/fault renders red, failsafe/warning renders amber, MAVLink/info renders blue — all visually distinct
- [x] 3.2 Confirm disabled states (`.gear-btn:disabled`, `.slider:disabled`, `.upload-btn:disabled`) remain visibly de-emphasized
- [x] 3.3 Grep `data/index.html` for stray color literals; only intentionally-kept `--warning`/`--info`/`--danger`/`--success` token definitions should hold raw values

## 4. Deploy and visually verify
- [x] 4.1 Run `pio run -t uploadfs` to flash the updated filesystem image
- [x] 4.2 Connect a phone to the `QuadBike-Control` AP, open `http://192.168.4.1`, and confirm the Sage Garden dark theme renders correctly with no external font/network requests and all controls legible

## 5. Validate
- [x] 5.1 Run `openspec validate apply-sage-garden-theme --strict` and resolve any issues
