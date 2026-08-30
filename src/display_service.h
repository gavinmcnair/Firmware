#ifndef DISPLAY_SERVICE_H
#define DISPLAY_SERVICE_H

#include <stdint.h>
#include <stdbool.h>

#define OPENDISPLAY_DECOMPRESSION_CHUNK_SIZE 2048

// EPD panel power state machine states. The enum type + keep-alive constant live
// in this shared header (included by both main.cpp via main.h and by
// display_service.cpp) so the type is visible to every TU. The single
// source-of-truth VARIABLES (pwrmgmState / pwrmgmOffDeadlineMs / pwrmgmLock) are
// DEFINED in main.h next to displayPowerState and externed in display_service.cpp.
// PWR_OFF must be 0: BSS-zero after boot / ESP32 deep-sleep wake == rail off.
enum PwrMgmState : uint8_t { PWR_OFF = 0, PWR_WARM = 1, PWR_ACTIVE = 2 };
#define EPD_KEEPALIVE_MAX_S 30   // hard cap on power_option.screen_timeout_seconds (clamped, not rejected)

// EPD panel power session (keep-alive) cross-TU API. Acquire/Release are
// file-static in display_service.cpp (they own the ACTIVE<->WARM transitions and
// need panel-init knowledge); these are the public entry points.
void epdSessionForceOff(void);   // power the panel fully down now (idempotent)
void epdSessionTick(void);       // millis()-poll from loop()/idleDelay(): expire keep-alive
bool epdSessionIsWarm(void);     // true when the panel is powered-idle (PWR_WARM)

bool fastepd_driver_used(void);
int mapEpd(int id);
bool waitforrefresh(int timeout);
float readBatteryVoltage();
float readChipTemperature();
void updatemsdata();
void initio();
void initDataBuses();
/** True when data_bus[0] is a configured I2C bus (pin_1/pin_2 not 0xFF). */
bool openDisplayI2cBusConfigured(void);
/** Re-apply I2C from data_bus[0] when set; else Wire.begin(). Call before TCON/touch on shared bus. */
void initOrRestoreWireForOpenDisplay(void);
/** Select data_buses[bus_id] on Wire (bus_id 0xFF → 0). Switches pins when multiple I2C buses are configured. */
bool initOrRestoreWireForBus(uint8_t bus_id);
/** Call after Wire.end() so the next touch/sensor access re-inits the bus. */
void invalidateOpenDisplayWire(void);
void scanI2CDevices();
void initSensors();
void initAXP2101(uint8_t busId);
void readAXP2101Data();
void powerDownAXP2101();
void initDisplay();
void writeTextAndFill(const char* text);
void handleDirectWriteStart(uint8_t* data, uint16_t len);
void handleDirectWriteData(uint8_t* data, uint16_t len);
// Consumes one direct-write compressed payload into the panel controller. Returns
// false on overflow/decompress failure; the CALLER owns ACK/NACK emission.
bool handleDirectWriteCompressedData(uint8_t* data, uint16_t len);
void cleanupDirectWriteState(bool refreshDisplay);
// PIPE_WRITE (0x0080-0x0082) sliding-window handlers + state reset.
void handlePipeWriteStart(uint8_t* data, uint16_t len);
void handlePipeWriteData(uint8_t* data, uint16_t len);
void handlePipeWriteEnd(uint8_t* data, uint16_t len);
void resetPipeWriteState(void);

// LOCAL FORK DIVERGENCE (PSRAM slot storage, not upstream -- see
// opendisplay_protocol.h's PIPE_FLAG_SLOT_TARGET). Copies slot_index's stored
// compressed bytes to the panel controller and refreshes -- no BLE traffic at
// all, safe to call from a local button-press handler. Returns false (no-op)
// if slot storage is disabled on this board, the index is out of range or
// unwritten, or a BLE transfer is currently active (transferActive()).
bool odDisplaySwitchToSlot(uint8_t slot_index);

// Scans slots[] for the next (direction > 0) or previous (direction < 0)
// populated (valid) slot from the current position, wrapping at OD_SLOT_COUNT,
// skipping unpopulated slots, and switches to it. Slot 0 is never a landing
// spot for this scan (valid or not) -- it's the reserved index/home page,
// reachable only via odDisplayJumpToSlot. No-op (returns false) if zero
// non-zero slots are populated or slot storage is disabled on this board.
bool odDisplayCycleSlot(int8_t direction);

// Jumps directly to slot_index (used for KEY3 -> slot 0, the reserved
// index/home page, and now the CMD_SLOT_SWITCH BLE handler below). Unlike
// odDisplayCycleSlot, this doesn't search for a populated slot -- it fails
// cleanly (false, no-op) if slot_index isn't valid, same as
// odDisplaySwitchToSlot.
bool odDisplayJumpToSlot(uint8_t slot_index);

// CMD_SLOT_SWITCH (0x0084) handler -- LOCAL FORK DIVERGENCE, not upstream
// (see opendisplay_protocol.h). The server-side equivalent of a button press:
// switches the panel to data[0] (slot_id) via odDisplayJumpToSlot() and ACKs
// or NACKs accordingly. See the opcode's doc block for the exact wire format.
void handleSlotSwitch(uint8_t* data, uint16_t len);

// Reserve the PIPE reorder queue (33 x 252 B on S3) in PSRAM. No-op unless the
// target has both a WiFi surface and PSRAM; idempotent; never freed. Call from
// setup() before BLE accepts commands. A failure here is defective hardware and is
// logged, not handled -- see odDisplayReserveBuffers() in display_service.cpp.
void odDisplayReserveBuffers(void);
// True while a PIPE_WRITE stream is active (mid-transfer log suppression, resets).
bool pipeWriteActive(void);
// True while ANY transfer (DIRECT / PIPE / PARTIAL) is streaming. Use this for
// "is the device busy" gates; test an individual flag only when the logic is
// genuinely specific to that one transfer type.
bool transferActive(void);
void handleDirectWriteEnd(uint8_t* data, uint16_t len);
// True while an image push is mid-stream and the per-frame command/ack logging
// should be suppressed (chunk 1 still logs in full; the meter covers the rest).
bool imageWriteLogQuietCmd(void);
bool imageWriteLogQuietAck(void);
bool imageWriteLogQuietFrame(const uint8_t* data, uint16_t len);
extern volatile bool epdRefreshInProgress;
/**
 * Close the refresh bracket: clears epdRefreshInProgress AND re-stamps the owner's
 * activity clock. Every refresh path must end through this rather than assigning
 * the flag, or that path silently loses the R4 refresh exclusion and can drop an
 * engaged client the moment loop() resumes.
 */
void endRefresh(void);
void handlePartialWriteStart(uint8_t* data, uint16_t len);
// Both transfer watchdogs, together. They live beside the state they terminate
// rather than in loop(): pipeState is reachable from main.cpp via
// resetPipeWriteState(), so this is cohesion rather than necessity -- but keeping
// the two apart is exactly how the direct-write timeout came to tear down the
// panel and leave its enclosing PIPE session running.
void checkTransferTimeouts(void);
void cleanupPartialWriteOnDisconnect(void);
// Origin (see commandOrigin()) of the transport that opened the in-flight transfer.
// A disconnect must only tear down a session its own transport owns.
uint8_t transferSessionOrigin(void);
int getplane();
int getBitsPerPixel();

#endif
