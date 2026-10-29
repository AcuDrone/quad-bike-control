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
