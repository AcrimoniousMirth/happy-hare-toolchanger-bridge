# Happy Hare Toolchanger Bridge

A companion library and configuration set for integrating **Happy Hare MMU** with **Klipper-Toolchanger** (and multi-toolhead setups).

## Purpose

Standard Happy Hare is designed for a single physical extruder. This bridge allows Happy Hare to:
1.  **Dynamically switch extruders**: Swap the active `[extruder]` and `[mmu_extruder_stepper]` at runtime.
2.  **Shared Sensors**: Dynamically swap `filament_switch_sensor` objects (extruder, toolhead, and tension sensors) based on the active toolhead.
3.  **Cross-Toolhead Loading**: Coordinates filament unloading from one toolhead and loading into another during a physical tool change.

## Implementation Design (Why and Where)

### 1. Deterministic Loading Order (`mmu.cfg`)
Klipper's `[include]` system can be non-deterministic when using globs like `*.cfg`. This bridge requires a specific sequence:
- **Base Macros** must be registered before the bridge can hook into them.
- **Hardware Pins** must be defined before sensors are initialized.

**Recommendation**: Use a "Master Config" file (see `examples/mmu_example.cfg`) and include it explicitly in your `printer.cfg`.

### 2. Hooks vs. Renames (`mmu_toolchanger_bridge.cfg`)
Traditional G-code overrides use `rename_existing`. However, in complex setups, this often leads to "Command not found" errors due to registration timing.

**This bridge uses Happy Hare's built-in Extension Hooks** (e.g., `user_pre_initialize_extension`). 
- **Why?** It is 100% stable and survives Happy Hare software updates. 
- **Scripted Fix**: We use a `[delayed_gcode]` to automatically inject our bridge logic into Happy Hare's startup sequence, even if your base configuration files get reverted during an update.

## Installation

1.  Clone this repository to your printer:
    ```bash
    git clone https://github.com/AcrimoniousMirth/happy-hare-toolchanger-bridge.git ~/happy-hare-toolchanger-bridge
    ```
2.  Run the installation script:
    ```bash
    cd ~/happy-hare-toolchanger-bridge && ./install.sh
    ```
3.  Add the bridge to your MMU configuration (see `examples/`).

## Configuration Prerequisites

### Sensor Naming
The bridge expects sensors to follow a specific naming convention:
- **T0 Path**: `mmu_extruder_0`, `mmu_toolhead_0`, `mmu_tension_0`
- **T1 Path**: `mmu_extruder_1`, `mmu_toolhead_1`, `mmu_tension_1`

### Hardware Config
In your `mmu_hardware.cfg`, leave the following blank:
```ini
[mmu_sensors]
extruder_switch_pin: 
toolhead_switch_pin: 
```
The bridge will dynamically swap your toolhead-specific sensors into these slots in real-time.
