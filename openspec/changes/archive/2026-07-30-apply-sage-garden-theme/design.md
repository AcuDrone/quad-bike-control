## Context
The web UI is a single self-contained file, `data/index.html`, served from LittleFS on the ESP32 with no internet access. All styling lives in one inline `<style>` block plus ~92 inline `style="..."` attributes and a few JavaScript lines that assign `element.style.color`. Colors today are hardcoded literals (blue gradient background `#1e3c72`→`#2a5298`, green `#4caf50`, red `#f44336`, orange `#ff9800`, blue `#2196f3`, plus neutral grays and translucent whites).

This change adopts the **tweakcn "Sage Garden" dark palette** as the single theme. Source registry URL: `https://tweakcn.com/r/themes/sage-garden.json` (shadcn registry format). Only the dark palette is used — there is no light mode and no `prefers-color-scheme` switching.

## Goals / Non-Goals
- Goals: centralize color decisions in `:root` CSS custom properties; map Sage Garden tokens onto every existing UI element; keep vehicle-safety status colors clearly distinguishable; no external font/CDN dependency.
- Non-Goals: layout redesign, DOM/markup restructuring, behavior/logic changes, light-mode support, adding new components.

## Extracted Sage Garden dark palette (authoritative values)
Values are copied verbatim from the tweakcn registry (`oklch`). Approximate sRGB hex equivalents are provided as an optional fallback for tooling/older engines; the `oklch` values are authoritative. Modern mobile browsers (the only clients, via the AP) support `oklch()` natively, so `oklch` may be used directly.

| Token | oklch (authoritative) | approx hex |
|-------|-----------------------|-----------|
| `--background` | `oklch(0.1448 0 0)` | `#0a0a0a` |
| `--foreground` | `oklch(0.9702 0 0)` | `#f5f5f5` |
| `--card` | `oklch(0.1822 0 0)` | `#121212` |
| `--card-foreground` | `oklch(0.9702 0 0)` | `#f5f5f5` |
| `--popover` | `oklch(0.1822 0 0)` | `#121212` |
| `--popover-foreground` | `oklch(0.9702 0 0)` | `#f5f5f5` |
| `--primary` | `oklch(0.6333 0.0309 154.9039)` | `#7c9082` |
| `--primary-foreground` | `oklch(0 0 0)` | `#000000` |
| `--secondary` | `oklch(0.2178 0 0)` | `#1a1a1a` |
| `--secondary-foreground` | `oklch(0.9702 0 0)` | `#f5f5f5` |
| `--muted` | `oklch(0.2178 0 0)` | `#1a1a1a` |
| `--muted-foreground` | `oklch(0.7058 0 0)` | `#a0a0a0` |
| `--accent` | `oklch(0.3709 0.0248 153.9823)` | `#36443a` |
| `--accent-foreground` | `oklch(0.9702 0 0)` | `#f5f5f5` |
| `--destructive` | `oklch(0.6368 0.2078 25.3313)` | `#ef4444` |
| `--destructive-foreground` | `oklch(1.0000 0 0)` | `#ffffff` |
| `--border` | `oklch(0.2850 0 0)` | `#2a2a2a` |
| `--input` | `oklch(0.1822 0 0)` | `#121212` |
| `--ring` | `oklch(0.6333 0.0309 154.9039)` | `#7c9082` |
| `--radius` | `0.35rem` | — |
| `--chart-1` | `oklch(0.6333 0.0309 154.9039)` | `#7c9082` |
| `--chart-2` | `oklch(0.7209 0.0489 120.9474)` | `#a0aa88` |
| `--chart-3` | `oklch(0.6744 0.0427 136.0110)` | `#8b9d83` |
| `--chart-4` | `oklch(0.5510 0.0234 264.3637)` | `#6b7280` |
| `--chart-5` | `oklch(0.5096 0.0289 152.3460)` | `#5a6b5e` |
| `--sidebar` | `oklch(0.1684 0 0)` | `#0f0f0f` |
| `--sidebar-foreground` | `oklch(0.9702 0 0)` | `#f5f5f5` |
| `--sidebar-primary` | `oklch(0.6333 0.0309 154.9039)` | `#7c9082` |
| `--sidebar-accent` | `oklch(0.2178 0 0)` | `#1a1a1a` |
| `--sidebar-border` | `oklch(0.2850 0 0)` | `#2a2a2a` |

Theme font names (informational — see Fonts decision below, NOT loaded): `--font-sans: Antic, ui-sans-serif, sans-serif, system-ui`; `--font-serif: Signifier, Georgia, serif`; `--font-mono: JetBrains Mono, Courier New, monospace`.

## Decisions

### Decision: Add semantic status tokens beyond the raw shadcn set
Sage Garden ships `--primary` (sage green), `--destructive` (red), `--accent` (muted green) but no dedicated warning/info tokens. The vehicle UI needs three clearly distinct safety states plus an info accent. Add two extra semantic tokens in `:root` so status colors stay centralized and distinguishable:
- `--success: var(--primary)` — sage green; used for connected / web-active / active-gear / OK calibration.
- `--warning: #ff9800` — amber; retained for failsafe / warning banners (deliberately kept outside the sage/red pair so it reads as caution, not OK and not fault).
- `--info: #2196f3` — blue; retained for MAVLink-active status, file-input, and calibration "Set" buttons.
- `--danger: var(--destructive)` — red; used for disconnected / sensor fault / error calibration.

Rationale: safety-critical UIs must not blur warning and error. `--primary` (sage) and `--destructive` (red) are far apart in hue, and amber `--warning` sits clearly between them. Keeping amber/blue as literals (not remapped to sage chart colors) preserves the existing, learned meaning of those states.

### Decision: Token → existing UI element mapping
| UI element (current color) | New token |
|----------------------------|-----------|
| `body` background gradient (`#1e3c72`→`#2a5298`) | `--background` (flat) or a subtle gradient of `--background`/`--sidebar` |
| Body / heading text (`#fff`, `#ffffff`) | `--foreground` |
| `.card`, `.status-item`, `.telemetry-item` surfaces (`rgba(255,255,255,0.1)`, `rgba(0,0,0,0.2)`) | `--card` (+ `--border` for edges) |
| Card / section borders (`rgba(255,255,255,0.2/0.3)`) | `--border` |
| Muted labels / secondary text (`#aaa`, `opacity`) | `--muted-foreground` |
| Neutral panels (`#2a2a2a`, `#555`, `#444`) | `--secondary` / `--muted` / `--border` |
| Primary action buttons, `.upload-btn`, slider thumb, progress fill, active gear (`#4caf50`, `#45a049`, `#8bc34a`) | `--primary` (text on it: `--primary-foreground`) |
| Connected / web-active / OK status (`#4caf50`) | `--success` |
| Disconnected / error / fault (`#f44336`) | `--danger` |
| Failsafe / warning banner (`#ff9800`, `#ffa500`, `#e65100`, `#ff6d00`) | `--warning` |
| MAVLink-active / info / file-input / "Set" buttons (`#2196f3`) | `--info` |
| Steering test / jog / nudge buttons (`#607d8b`, `#795548`, `#9c27b0`) | `--secondary` / `--accent` (grouped as neutral secondary actions) |
| Slider / progress track (`rgba(0,0,0,0.3)`) | `--input` / `--muted` |
| `border-radius` literals (`8px`, `12px`, etc.) | keep as-is OR optionally reference `--radius`; radius is out of strict scope, colors are the priority |
| JS-set `statusEl.style.color = '#4caf50' / '#f44336'` | assign `var(--success)` / `var(--danger)` strings |

Translucent-white overlays (`rgba(255,255,255,0.1)`) become solid `--card`/`--secondary` surfaces since the flat dark background no longer benefits from glass/backdrop-blur layering; `backdrop-filter` may remain but has little visual effect over a flat background.

### Decision: Fonts — system stack only, no external loading
The theme references Antic / Signifier / JetBrains Mono. These MUST NOT be pulled from Google Fonts or any CDN — the device has no internet and serves everything from LittleFS. Keep the existing system font stack (`-apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, ...`). Optionally define `--font-sans`/`--font-mono` variables whose values are pure system-stack fallbacks (omitting the proprietary names, or listing them last where they simply no-op if absent). No `@font-face` with remote `url()`, no `<link>` to fonts. Embedding font files into LittleFS is explicitly out of scope for this change.

## Risks / Trade-offs
- `oklch()` support: All modern mobile browsers support it; the approximate hex column is the documented fallback if a target browser is found lacking. Mitigation: hex values recorded above so no recomputation is needed.
- Losing status legibility: mitigated by the dedicated `--success`/`--warning`/`--info`/`--danger` tokens with high hue separation.
- Missed literal: with ~92 inline styles plus JS color assignments, a stray literal could be overlooked. Mitigation: tasks refactor section by section and a final grep confirms no stray non-token color literals remain (except intentionally kept `--warning`/`--info` definitions).

## Migration Plan
Not a data/behavior migration. Deploy by reflashing the filesystem image: `pio run -t uploadfs`. Rollback = re-flash the previous `index.html`. No firmware (`pio run` app image) change required.

## Open Questions
- None blocking. Whether to also tokenize `border-radius` via `--radius` is optional polish and can be deferred.
