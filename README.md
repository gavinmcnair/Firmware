# Open Display Firmware for NRF52840 and ESP32

Firmware and tools for BLE (Bluetooth Low Energy) based Open Display tags.

## Getting Started

To quickly get started, visit the following resources:

- [Install](https://opendisplay.org/firmware/install/) — upload firmware to supported boards
- [Configure](https://opendisplay.org/firmware//config/) — configure your BLE tag settings  
- [Display](https://opendisplay.org/firmware/display/) —  test your display and upload images

## Community

Join the Open Display community on Discord for help, discussions, and development updates:  
[Open Display Discord Server](https://discord.gg/wgQ8XsgMkv)

## Supported Hardware

Currently, displays based on the following ICs should be supported:

- NRF52840
- ESP32-S3
- ESP32-C6
- ESP32-C3

## Persistent slot storage (this fork)

> **LOCAL FORK DIVERGENCE** — this feature and its wire-protocol extension
> (`PIPE_FLAG_SLOT_TARGET`) are specific to this fork and are not part of the
> canonical OpenDisplay protocol upstream.

PSRAM-equipped boards (ESP32-S3/wrover-e) can hold several pre-rendered pages
at once in flash-backed on-device "slots" (LittleFS files on the existing data
partition, so they survive deep sleep and power loss) — pushed ahead of time
over BLE, switched between locally by KEY1/KEY2 (cycle) and KEY3 (jump to the
slot 0 index) with no BLE round trip at switch time; `CMD_SLOT_SWITCH`
(0x0084) is the server-driven equivalent. Slot capacity is derived at runtime
from the board's filesystem size (up to 100). See
[docs/pipe-write-protocol.md §10](docs/pipe-write-protocol.md#10-persistent-slot-storage-pipe_flag_slot_target--local-fork-divergence)
for the full wire format, validation, and switching behaviour.

[`gavinmcnair/opendisplay-pages`](https://github.com/gavinmcnair/opendisplay-pages)
is a Rust client built on this feature — a plugin-based BLE pusher that
renders independent pages (a live train-times board, an all-day weather
forecast, an auto-generated index of what's on which slot) and pushes each to
its own slot only when that page's content actually changed.
