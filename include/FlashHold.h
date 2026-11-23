#ifndef FLASH_HOLD_H
#define FLASH_HOLD_H

/**
 * @file FlashHold.h
 * @brief One loop-written flag: periodic flash writes wait while it is set (change
 * replace-aztec-brake-with-ibooster D9). VehicleController sets it every loop from the service
 * brake's requestActive(); the odometer (every 1 km) and the engine-hour meter (every 600 s) skip
 * their PERIODIC save while it is set and save on the first update after it clears. Explicit and
 * fail-safe saves ignore it. A flash erase stalls both cores 30-50 ms, and with them the CAN task
 * that sends the brake frames every 10 ms.
 */
namespace FlashHold {
inline volatile bool& flag() {
    static volatile bool f = false;
    return f;
}
inline void set(bool on) { flag() = on; }
inline bool active() { return flag(); }
}  // namespace FlashHold

#endif  // FLASH_HOLD_H
