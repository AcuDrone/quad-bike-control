#ifndef CHANGELOG_H
#define CHANGELOG_H

/**
 * @file Changelog.h
 * @brief User-facing release notes, shipped inside the firmware binary.
 *
 * Served verbatim by GET /api/changelog and rendered by the web UI ("What's new").
 * Lives in the firmware rather than in data/index.html so it can never drift from
 * FIRMWARE_VERSION: the firmware and the LittleFS image are flashed independently.
 *
 * Format: {"releases":[{version, date, uk:[...], en:[...]}, ...]}, newest first.
 * Keep entries short and plain — one line per change, read by the operator.
 * When bumping FIRMWARE_VERSION in Constants.h, add an entry here.
 */
static const char CHANGELOG_JSON[] = R"JSON({"releases":[
{"version":"1.5.31","date":"2026-11-13","uk":[
"Плавне відкриття газу тепер типово ВИМКНЕНО (0): газ відкривається миттєво, як до 1.5.16 — глухнення квадроцикла спричинив перегрів двигуна, а не різкий газ. Якщо на платі вже збережено швидкість (наприклад 60), вона лишається після оновлення; щоб вимкнути, задайте 0 у картці «Обмеження швидкості та кермо» й збережіть. Обмеження й далі доступне (30–100 %/с) для м’якшого відкриття газу. Потрібно залити оновлений образ LittleFS"
],"en":[
"Smooth throttle opening is now OFF by default (0): the throttle opens instantly, as before 1.5.16 — the quad stall was caused by engine overheating, not by snapping the throttle. A unit with a saved rate (e.g. 60) keeps it after the update; to turn it off, set 0 in the Speed limit and steering card and save. The limit is still available (30–100 %/s) for a softer opening. Upload the updated LittleFS image"
]},
{"version":"1.5.30","date":"2026-11-13","uk":[
"Наземна станція (Mission Planner, QGC, кнопки на кермі): передача, запалювання, фара і блокування тепер керуються командами з наземної станції (MAV_CMD_USER_2). Плата перевіряє всі блокування й відповідає на кожну команду: передача — лише з працюючим двигуном і не швидше 1,4 м/с (нейтраль — на будь-якій швидкості); запалювання OFF працює завжди, ACC/ON і блокування — лише коли керує MAVLink (не у failsafe і не з веб-керуванням); фара — завжди.",
"Пульт і наземна станція рівноправні: діє остання зміна. Перемикач на пульті застосовується лише тоді, коли його положення змінилося, тож команда з наземної станції не скасовується наступним кадром. Канал поза діапазоном 880–2160 мкс більше не вмикає N / OFF / вимикає фару — стан лишається.",
"Після завантаження, після failsafe і після виходу з веб-керування (або коли пульт повернувся при відкритій веб-сторінці) перший кадр із пульта знову виставляє передачу, запалювання, фару й блокування — якщо перемикач запалювання в ON, запускається автостарт.",
"Для наземної станції: можливості плати (QB_CAPS кожні 10 с), стан запалювання (IGN_STATE: 0 OFF, 1 ACC, 2 ON, 3 прокрутка стартером), запити версії прошивки й обладнання. Без цієї прошивки станція нових кнопок не показує."
],"en":[
"Ground station (Mission Planner, QGC, steering-wheel buttons): gear, ignition, headlight and diff lock are now commanded from the ground station (MAV_CMD_USER_2). The board checks every interlock and answers each command: gear only with the engine running and at no more than 1.4 m/s (neutral at any speed); ignition OFF always works, ACC/ON and the lock only while MAVLink is in control (not in failsafe or under web control); headlight always.",
"RC and ground station are equal: the last change wins. An RC switch is applied only when its position changes, so a ground-station command is not undone by the next frame. A channel outside 880–2160 µs no longer forces N / OFF / light off — the state is kept.",
"After boot, after a failsafe and after leaving web control (or when RC returns while the web page is open) the first RC frame sets gear, ignition, headlight and lock again — with the ignition switch in ON this starts the auto-crank.",
"For the ground station: board capabilities (QB_CAPS every 10 s), ignition state (IGN_STATE: 0 OFF, 1 ACC, 2 ON, 3 cranking), firmware and hardware version queries. Without this firmware the ground station shows no new buttons."
]},
{"version":"1.5.28","date":"2026-11-12","uk":[
"Квадроцикл: на лінії прийому від VESC керма (X6, GPIO5) увімкнено внутрішню підтяжку до +3,3 В. На X6 уже є зовнішні підтяжки через перетворювачі рівнів, тож поведінка не змінюється — внутрішня додана для однаковості з багі. Якщо підтяжку не вдалося ввімкнути, у журналі з’являється запис «VESC W rx_pullup»",
"CAN: при вимкненому запалюванні (OFF) мовчання ECU більше не вважається збоєм — контролер CAN не перезапускається кожні 15 с, у журналі немає «obd_silent», лічильник can_reinit не росте (як уже було для датчика кута керма). Після перемикання в ACC/ON відлік 15 с починається заново"
],"en":[
"Quad: the internal pull-up to +3.3 V is enabled on the steering VESC receive line (X6, GPIO5). X6 already has external pull-ups through the level shifters, so behaviour does not change — the internal one is added for parity with the buggy. If the pull-up cannot be enabled, the journal shows “VESC W rx_pullup”",
"CAN: with the ignition OFF a silent ECU is no longer treated as a fault — the CAN controller is not restarted every 15 s, no “obd_silent” in the journal, the can_reinit counter does not grow (as already done for the steering-angle sensor). After switching to ACC/ON the 15 s window starts afresh"
]},
{"version":"1.5.27","date":"2026-11-12","uk":[
"Веб-панель: підтвердження без спливаючих вікон браузера — «Задати нуль датчика», «Скинути до типових» (коди передач), «Очистити» (журнал подій) і «Завантажити прошивку» тепер підтверджуються другим натисканням тієї ж кнопки протягом 5 с; попередження показується в рядку стану картки. Раніше у вікні входу в мережу на телефоні або після «заборонити сторінці створювати діалоги» в Chrome кнопка мовчки нічого не робила. Результат завантаження прошивки теж показується в картці. Потрібно залити оновлений образ LittleFS",
"Калібрування керма: «Задати ліво», «Задати право» і «Задати нуль датчика» без з’єднання з платою тепер показують у рядку стану картки «Немає з'єднання з платою», а не мовчать"
],"en":[
"Web panel: confirmations without browser pop-ups — “Set sensor zero”, “Reset to defaults” (gear codes), “Clear” (event journal) and “Upload Firmware” are now confirmed by pressing the same button again within 5 s; the warning is shown in the card’s status line. Before, in the phone’s network sign-in window or after Chrome’s “prevent this page from creating additional dialogs” the button silently did nothing. The firmware upload result is also shown in the card. Upload the updated LittleFS image",
"Steering calibration: “Set Left”, “Set Right” and “Set sensor zero” now show “No connection to the board” in the card’s status line when the board is not connected, instead of doing nothing"
]},
{"version":"1.5.26","date":"2026-11-10","uk":[
"Журнал подій: нова картка «Журнал подій» у розділі «Система» — останні 100 подій (запуски й перезапуски, failsafe, режим керма, втрата зв’язку з VESC керма, збої CAN і шини реле, нестача пам’яті, знімок стану щохвилини) з фільтром за джерелом, кнопками «Завантажити» (весь журнал одним файлом, ≈ 1–2 доби) та «Очистити». Журнал зберігається у флеші й переживає оновлення прошивки",
"Журнал подій: запис у флеш типово ВИМКНЕНО — вмикається перемикачем «Запис у flash» у картці «Журнал подій» (налаштування зберігається після перезапуску). Поки запис вимкнено, журнал тримається лише в пам’яті (останні ≈ 4 КБ рядків) і зникає після вимкнення живлення",
"Самовідновлення: зв’язок із VESC керма більше не «зависає» на хвилини після одного пошкодженого байта (відновлюється з наступною відповіддю); датчик кута керма на CAN перезапускається сам, якщо замовк при ввімкненому запалюванні (ACC/ON), а ECU відповідає; шина I2C плати реле звільняється й перезапускається сама. Кожне таке відновлення записується в журнал, лічильники показано в картці",
"Потрібно залити оновлений образ LittleFS. Увага: заливання образу файлової системи (uploadfs або оновлення LittleFS з веб-сторінки) стирає журнал — спершу завантажте його"
],"en":[
"Event journal: new “Event journal” card in the System section — the last 100 events (boots and resets, fail-safe, steering mode, steering VESC link loss, CAN and relay-bus faults, low memory, a status snapshot every minute) with a source filter, “Download” (the whole journal as one file, ≈ 1–2 days) and “Clear”. The journal is kept in flash and survives a firmware update",
"Event journal: writing to flash is OFF by default — switch it on with “Write to flash” in the “Event journal” card (the setting survives a reboot). While it is off, the journal is kept only in RAM (the last ≈ 4 KB of lines) and is lost at power-off",
"Self-recovery: the steering VESC link no longer stays stuck for minutes after one corrupted byte (it recovers with the next reply); the CAN steering-angle sensor is restarted automatically when it goes silent with the ignition on (ACC/ON) while the ECU still answers; the relay board I2C bus frees and restarts itself. Every such recovery is written to the journal and counted in the card",
"Upload the updated LittleFS image. Note: uploading the filesystem image (uploadfs or a LittleFS update from the web page) erases the journal — download it first"
]},
{"version":"1.5.24","date":"2026-11-07","uk":[
"Реле: підтримка плати реле XL9535-K8V5 (адреса 0x20–0x27) поряд із платою PCA9557 (0x1F). Плата визначається автоматично під час запуску; нова картка «Плата реле» в розділі «Система» дозволяє вибрати Авто / PCA9557 / XL9535 (діє після перезавантаження) і показує активну плату та помилку. Якщо відповідають обидві плати, жодна або кілька адрес XL9535 — жодне реле не вмикається. Кожне перемикання реле тепер перевіряється за станом виводів, а після збою живлення плати реле відновлюються самі приблизно за 1 с. Потрібно залити оновлений образ LittleFS"
],"en":[
"Relays: support for the XL9535-K8V5 relay board (address 0x20–0x27) alongside the PCA9557 board (0x1F). The board is detected automatically at boot; the new “Relay board” card in the System section selects Auto / PCA9557 / XL9535 (applies after reboot) and shows the active board and any error. If both boards, no board or several XL9535 addresses answer, no relay is switched on. Every relay write is now verified on the pins, and after a relay-board power glitch the relays recover by themselves within about 1 s. Upload the updated LittleFS image"
]},
{"version":"1.5.23","date":"2026-11-06","uk":[
"Гальмо: нова картка «Гальмо» — час повного ходу актуатора (повзунок 1000–3000 мс, типово 1500) і сила утримання (0–100, типово 30, 0 = без утримання) налаштовуються з веба для кожної машини окремо; показує стан кінцевика; плитка «Гальмо» на панелі тепер показує оцінене положення гальма замість 0. Потрібно залити оновлений образ LittleFS"
],"en":[
"Brake: new “Brake” card — the actuator full travel time (slider 1000–3000 ms, default 1500) and the hold drive (0–100, default 30, 0 = no hold push) are set from the web, per vehicle; it shows the endstop state; the panel “Brake” tile now shows the estimated brake position instead of 0. Upload the updated LittleFS image"
]},
{"version":"1.5.22","date":"2026-11-06","uk":[
"Запуск двигуна: стартер крутить до 5 с за одну спробу (було 2 с) і, як і раніше, відпускається одразу, щойно двигун завівся"
],"en":[
"Engine start: the starter cranks for up to 5 s per attempt (was 2 s) and, as before, is released immediately once the engine starts"
]},
{"version":"1.5.21","date":"2026-11-06","uk":[
"WiFi: назва мережі тепер унікальна для кожної плати — QuadBike-Control-XXXX (XXXX — останні 4 символи MAC чипа), тож кілька машин поруч не плутаються; після прошивки один раз підключіть телефон до нової мережі. Ім'я OTA теж з суфіксом (quadbike-control-xxxx), адреса 192.168.10.1 без змін"
],"en":[
"WiFi: the network name is now unique per board — QuadBike-Control-XXXX (XXXX = the last 4 hex digits of the chip MAC), so several vehicles nearby are no longer confused; after flashing, join the new network on the phone once. The OTA hostname gets the same suffix (quadbike-control-xxxx); the address 192.168.10.1 is unchanged"
]},
{"version":"1.5.20","date":"2026-11-06","uk":[
"Кермо: прибрано ривки на повороті (буст додається поверх мінімальної потужності, тримається до 2° від цілі й лише тоді скидається, мінімальна потужність 85) і недоїзд до центру на 0,3–0,4° (зупинка в межах 0,2°)"
],"en":[
"Steering: no stop-go jerks on a turn (the anti-stall boost adds on top of the duty floor, is held until 2° from the target and only then is shed, floor 85) and no 0.3–0.4° undershoot of centre (parks within 0.2°)"
]},
{"version":"1.5.19","date":"2026-11-06","uk":[
"Доводка центру: захист від петлі (переліт → перезапуск → знову імпульс); після занадто сильного імпульсу доводка чекає нової команди стіка"
],"en":[
"Steering park correction: loop protection (overshoot → restart → pulse again); after a too-strong pulse the correction waits for the next stick command"
]},
{"version":"1.5.18","date":"2026-11-06","uk":[
"Доводка керма в центр: якщо після повороту колесо зупинилось за 0,4–1,0° від цілі, прошивка через ≈1 с доводить його короткими імпульсами до 0,2° (до 5 імпульсів); налаштовується в новій картці «Доводка центру керма» (можна вимкнути)"
],"en":[
"Steering park correction: when the wheel parks 0.4–1.0° off target after a turn, the firmware finishes the move after ≈1 s with short pulses to within 0.2° (up to 5 pulses); set it in the new “Steering park correction” card (can be switched off)"
]},
{"version":"1.5.17","date":"2026-11-06","uk":[
"Поле точного кута керма на телефоні приймає від'ємні (ліві) кути: кнопка «±» змінює знак, а клавіатура тепер має мінус"
],"en":[
"The exact steering angle field accepts negative (left) angles on the phone: a “±” button flips the sign and the keyboard now has a minus key"
]},
{"version":"1.5.16","date":"2026-11-06","uk":[
"Плавне відкриття газу проти глухнення двигуна: газ відкривається не швидше заданої швидкості (типово 60 %/с — від холостих до повного за 1,7 с), скидання газу завжди миттєве; налаштовується у веб-панелі (картка «Обмеження швидкості та кермо», 0 = вимкнено, 30–100 %/с)"
],"en":[
"Smooth throttle opening against engine stalls: the throttle opens no faster than a set rate (default 60 %/s — idle to full in 1.7 s), closing is always instant; set it from the web (Speed limit and steering card, 0 = off, 30–100 %/s)"
]},
{"version":"1.5.15","date":"2026-11-06","uk":[
"Перемикання передач швидше: очікування в крайньому положенні та пауза між кроками закінчуються, щойно датчик ≈0,2 с стабільно показує потрібну передачу (N→L ≈2,6 с замість ≈6,7 с); без датчика — як раніше (2 с і 1 с), перегазовка/холостий хід тримаються лише на час очікування"
],"en":[
"Faster gear changes: the overshoot wait and the pause between steps end as soon as the gear sensor steadily reports the expected gear for ≈0.2 s (N→L ≈2.6 s instead of ≈6.7 s); without the sensor they time out as before (2 s and 1 s), and the RPM boost / idle hold only lasts for the wait"
]},
{"version":"1.5.14","date":"2026-11-06","uk":[
"Передача звіряється з датчиком положення: при старті вона береться з датчика (або зі збереженого стану, якщо датчик мовчить; інакше — один повний крок у нейтраль замість сліпого стрибка), а якщо в спокої датчик ≈1 с стабільно показує іншу передачу — прошивка приймає її; до ініціалізації передача показується як «?» і газ обмежено"
],"en":[
"The gear is reconciled with the gear sensor: at start it comes from the sensor (or the saved state if the sensor is silent; otherwise one full step into neutral instead of a blind jump), and when idle the firmware adopts the gear the sensor steadily reports for ≈1 s; until initialised the gear shows as “?” and throttle is capped"
]},
{"version":"1.5.13","date":"2026-11-05","uk":[
"Перегазовку при перемиканні передач можна вимкнути у веб-панелі (картка «Положення передач»); коли вимкнено, газ на час перемикання тримається на холостому ходу"
],"en":[
"The gear-change RPM boost can be switched off from the web (Gear Position Defaults card); when off, the throttle is held at idle while the gear shifts"
]},
{"version":"1.5.12","date":"2026-11-05","uk":[
"Діагностика керма через USB: консоль на USB-порту (скрипт tools/steer_diag.py) вимірює мінімальну потужність зрушення, вибіг, кут за імпульс і точність повернення в центр; поки вона увімкнена, інші команди керма не виконуються (лише на місці)",
"Налаштування керма за вимірами на машині: зона зупинки 0,4°, повторний рух від 1,0°, мінімальна потужність 80 (зрушення) / 70 (рух), точний імпульс 100 × 100 мс, ручний рух 120",
"Веб-панель: швидкість провороту 80–255 (типово 120), поле точного кута керма з кнопками «Перейти» та «0°», один Nudge ≈ 0,4°",
"Плавність обмежувача швидкості (як швидко він прибирає й повертає газ) налаштовується у веб-панелі: 10–200 %/с, типово 80"
],"en":[
"USB steering diagnostics: a console on the USB port (script tools/steer_diag.py) measures the break-away duty, coast, degrees per pulse and centring accuracy; while it is on, other steering commands are ignored (standing still only)",
"Steering tuning from measurements on the vehicle: park window 0.4°, restart beyond 1.0°, duty floors 80 (break-away) / 70 (tracking), nudge 100 × 100 ms, jog 120",
"Web UI: jog speed 80–255 (default 120), exact target angle input with Go and 0° buttons, one nudge ≈ 0.4°",
"Speed limiter throttle rate (how fast it closes and returns the throttle) adjustable from the web: 10–200 %/s, default 80"
]},
{"version":"1.5.7","date":"2026-11-03","uk":[
"Діагностика: кнопка «Сніфер CAN (10 с)» показує всі ID на шині CAN з лічильниками й даними (лише на місці)"
],"en":[
"Diagnostics: a “CAN sniffer (10 s)” button lists every ID on the CAN bus with counts and data (standing still only)"
]},
{"version":"1.5.6","date":"2026-11-03","uk":[
"Проба ЕБК показує сирі байти відповідей PID"
],"en":[
"ECU probe shows raw PID reply bytes"
]},
{"version":"1.5.5","date":"2026-11-03","uk":[
"Кермо від автопілота тепер плавно стежить за стіком, без ривків",
"Під час стеження мотор керма працює на нижчій потужності, повна потужність лише для зрушення з місця",
"Повторне калібрування не потрібне"
],"en":[
"Autopilot steering now follows the stick smoothly, without jerks",
"While tracking the steering motor runs at a lower minimum power; full break-away power only when starting from rest",
"No recalibration required"
]},
{"version":"1.5.4","date":"2026-11-02","uk":[
"Імпульси на оберт колеса тепер можна виміряти з веб-панелі (прокрутити колесо N обертів); картка датчика швидкості розділена на «Датчик швидкості» та «Обмеження швидкості та кермо»"
],"en":[
"Wheel pulses per revolution can now be measured from the web portal (turn the wheel N times); the speed sensor card is split into “Speed sensor” and “Speed limit and steering”"
]},
{"version":"1.5.3","date":"2026-11-02","uk":[
"Обмежувач швидкості тепер сам утримує ліміт: якщо швидкість вища — газ зменшується далі, аж до холостого ходу",
"Мінімальний газ обмежувача налаштовується в картці «Датчик швидкості» (0–20 %, типово 0 %)",
"На низьких лімітах газ знімається ближче до ліміту, а не з половини",
"Повторне калібрування не потрібне"
],"en":[
"The speed limiter now holds the limit itself: above it, throttle keeps closing down to idle",
"The limiter's minimum throttle is set in the Speed sensor card (0–20 %, default 0 %)",
"At low limits, throttle is withdrawn close to the limit instead of from half of it",
"No recalibration required"
]},
{"version":"1.5.2","date":"2026-10-31","uk":[
"Квадроцикл: кут керма тепер читається з датчика Bosch на шині CAN; AS5600 більше не використовується",
"Нова кнопка «Задати нуль датчика»; калібруються лише межі «ліво» та «право»",
"Без датчика кермо керується стіком як швидкістю (тільки MANUAL); перед рухом стік один раз у центр",
"Веб-панель — сервісна консоль за адресою 192.168.10.1 (було 192.168.4.1)",
"Перевірка реле у веб-панелі (лише при вимкненому запалюванні та відпущеному блокуванні колеса)",
"Калібрування газу приймає дзеркальний серво",
"Блокування переднього колеса, реле, датчики передач і VESC на X6 — без змін",
"Потрібне повторне калібрування керма: нуль датчика, потім «ліво» та «право»",
"Мінімальний кут ліміту керма від нуля датчика зменшено з 30° до 20°"
],"en":[
"Quad: steering angle now comes from the Bosch sensor on the CAN bus; the AS5600 is no longer used",
"New \"Set sensor zero\" button; only the left and right limits are calibrated",
"Without the sensor the stick steers by rate (MANUAL only); centre the stick once before moving",
"Web portal is a service console at 192.168.10.1 (was 192.168.4.1)",
"Relay test in the web portal (ignition off and wheel lock released only)",
"Throttle calibration accepts a mirrored servo",
"Front-wheel lock, relays, gear sensors and the VESC on X6 are unchanged",
"Steering recalibration required: sensor zero, then left and right",
"Minimum steering limit angle from the sensor zero lowered from 30° to 20°"
]},
{"version":"1.5.1","date":"2026-10-29","uk":[
"Напрямок датчика кута визначається автоматично при калібруванні лімітів; дзеркальний монтаж більше не потребує перемонтажу",
"Після першого ліміту перевірте: Jog ▶ збільшує кут",
"Повторне калібрування не потрібне"
],"en":[
"Steering sensor direction is detected at limit capture; a mirrored mount no longer needs re-fitting",
"After the first limit, check that Jog ▶ increases the angle",
"No recalibration required"
]},
{"version":"1.5.0","date":"2026-10-28","uk":[
"Без датчика кута кермо керується стіком як швидкістю: відхилення = обертання, центр = стоп; тільки в MANUAL",
"Перед початком руху стік потрібно один раз поставити в центр",
"Повзунок керма в веб-панелі блокується без датчика; Jog/Nudge працюють",
"Повторне калібрування не потрібне"
],"en":[
"Without the angle sensor the stick steers by rate: deflection = rotation, centre = stop; MANUAL only",
"Centre the stick once before the wheels will move",
"The web steering slider is locked without the sensor; Jog/Nudge still work",
"No recalibration required"
]},
{"version":"1.4.0","date":"2026-10-27","uk":[
"Веб-панель перероблена на сервісну консоль: розділи Діагностика, Калібрування, Тести, Система",
"Довідка до кожної картки за кнопкою ⓘ",
"Положення передач задаються одним повзунком і кнопками «Зберегти як»",
"Блок «Увага» показує низьку напругу VESC (<21.5 В), низьку напругу 12 В (<12.0 В) і втрату зв'язку з ЕБК",
"Повторне калібрування не потрібне"
],"en":[
"Web portal redesigned as a service console: Diagnostics, Calibration, Tests, System sections",
"Help for every card behind the ⓘ button",
"Gear positions are set with one slider and \"Save as\" buttons",
"The \"Attention\" box shows low VESC voltage (<21.5 V), low 12 V voltage (<12.0 V) and lost ECU link",
"No recalibration required"
]},
{"version":"1.3.0","date":"2026-10-27","uk":[
"Кут керма тепер читається з датчика Bosch на шині CAN; датчики AS5600 прибрано",
"Нова кнопка «Задати нуль датчика»: поточне положення коліс стає центром керма",
"Центр керма — це нуль датчика; калібруються лише межі «ліво» та «право», кожна зберігається одразу",
"Проворот і підштовхування керма працюють навіть без датчика (без захисту від заклинювання)",
"Датчик за межами діапазону (~600°) показує «перезапустіть машину»; автоматичне кермо зупиняється на 550°",
"Після оновлення потрібне повторне калібрування керма: нуль датчика (якщо треба), потім «ліво» та «право»",
"Адреса web-панелі змінена на 192.168.10.1 (було 192.168.4.1)"
],"en":[
"Steering angle now comes from the Bosch sensor on the CAN bus; the AS5600 sensors are removed",
"New \"Set sensor zero\" button: the current wheel position becomes the steering center",
"The steering center is the sensor zero; only Left and Right are captured, each saved at once",
"Steering jog and nudge work even without a sensor (no stall protection)",
"A sensor past its range (~600°) shows \"restart the vehicle\"; automatic steering stops at 550°",
"Steering must be recalibrated after this update: sensor zero (if needed), then Left and Right",
"Web portal address changed to 192.168.10.1 (was 192.168.4.1)"
]},
{"version":"1.2.0","date":"2026-10-19","uk":[
"Кермо баггі (ODES Junglecross 800): два датчики AS5600, положення на кілька обертів керма",
"Положення керма відновлюється після ввімкнення без пошуку нуля",
"Будь-яка помилка датчиків керма зупиняє кермо",
"Центр, ліву та праву межі керма можна фіксувати в будь-якому порядку (усі три за одне ввімкнення); після оновлення потрібне повторне калібрування керма",
"Реле баггі: 1 — запалювання, 2 — стартер, 3 — фара; реле 4–8 вимкнені (крім перевірки реле)",
"Блокування переднього колеса на баггі не встановлене — кнопка вимкнена",
"Перевірка всіх 8 реле у веб-інтерфейсі, лише при вимкненому запалюванні",
"Передача баггі визначається 3-бітним кодом на трьох входах; код видно в телеметрії",
"Коди датчика передач калібруються у веб-інтерфейсі: важіль у передачу → «Зберегти як»",
"Калібрування дроселя підтримує дзеркально встановлене серво: повний газ може бути меншим за холостий хід (µs)",
"VESC керма баггі підключено до роз'єму X1 (UART0); X6 звільнено"
],"en":[
"Buggy (ODES Junglecross 800) steering: two AS5600 sensors, multi-turn steering position",
"Steering position is recovered at power-up without homing",
"Any steering sensor fault stops the steering",
"Steering center, left and right can be captured in any order (all three in one power-on); steering must be recalibrated after this update",
"Buggy relays: 1 ignition, 2 starter, 3 front light; relays 4-8 off except in the relay test",
"Front wheel lock is not fitted on the buggy; its button is disabled",
"Relay test toggles for all 8 relays in the web UI, ignition OFF only",
"Buggy gear is read as a 3-bit code on three inputs; the code is shown in telemetry",
"Gear sensor codes are calibrated in the web UI: lever in gear, then \"Save as\"",
"Throttle calibration accepts a mirrored servo: full may be below idle (µs)",
"Buggy steering VESC moved to header X1 (UART0); X6 freed"
]},
{"version":"1.1.0","date":"2026-10-08","uk":[
"Нова плата керування Control_v0",
"Керування по MAVLink замість SBUS",
"Кермо на VESC, калібрування у веб-інтерфейсі",
"Повільніше кермо на швидкості в ручному режимі",
"Датчик швидкості колеса",
"Обмеження максимальної швидкості",
"Одометр, пробіг поїздки, мотогодини",
"Більше даних з двигуна, коди помилок DTC",
"Автовідновлення CAN-шини",
"Блокування переднього колеса, калібрування газу",
"Інтерфейс українською та англійською"
],"en":[
"New Control_v0 control board",
"MAVLink control instead of SBUS",
"VESC steering, calibration in the web UI",
"Slower steering at speed in manual mode",
"Wheel speed sensor",
"Maximum speed limit",
"Odometer, trip distance, engine hours",
"More engine data, DTC fault codes",
"CAN bus auto-recovery",
"Front wheel lock, throttle calibration",
"Ukrainian and English UI"
]},
{"version":"1.0.4","date":"2026-03-12","uk":[
"Перша версія: кермо, газ, гальмо, передачі, запалювання, світло"
],"en":[
"First release: steering, throttle, brake, gears, ignition, lights"
]}
]})JSON";

#endif // CHANGELOG_H
